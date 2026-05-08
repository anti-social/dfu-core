use super::*;
use std::convert::TryFrom;
use std::io::Cursor;
use std::prelude::v1::*;

struct Buffer<R: std::io::Read> {
    reader: R,
    buf: Box<[u8]>,
    level: usize,
}

impl<R: std::io::Read> Buffer<R> {
    fn new(size: usize, reader: R) -> Self {
        Self {
            reader,
            buf: vec![0; size].into_boxed_slice(),
            level: 0,
        }
    }

    fn fill_buf(&mut self) -> Result<&[u8], std::io::Error> {
        while self.level < self.buf.len() {
            let dst = &mut self.buf[self.level..];
            let r = self.reader.read(dst)?;
            if r == 0 {
                break;
            } else {
                self.level += r;
            }
        }
        Ok(&self.buf[0..self.level])
    }

    fn consume(&mut self, amt: usize) {
        if amt >= self.level {
            self.level = 0;
        } else {
            self.buf.copy_within(amt..self.level, 0);
            self.level -= amt;
        }
    }
}

fn wait_for_state<T, IO, E>(
    mut cmd: get_status::WaitState<T>,
    io: &IO,
    buffer: &mut [u8],
) -> Result<T, E>
where
    IO: DfuIo<Read = usize, Error = E>,
    E: From<Error>,
{
    loop {
        match cmd.next() {
            get_status::Step::Break(result) => return Ok(result),
            get_status::Step::Wait(gs, poll_timeout) => {
                std::thread::sleep(std::time::Duration::from_millis(poll_timeout));
                let (recv, mut control) = gs.get_status(buffer);
                let n = control.execute(io)?;
                cmd = recv.chain(&buffer[..n])??;
            }
        }
    }
}

/// Generic synchronous implementation of DFU.
#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
pub struct DfuSync<IO, E>
where
    IO: DfuIo<Read = usize, Write = usize, Reset = (), Error = E>,
    E: From<std::io::Error> + From<Error>,
{
    io: IO,
    dfu: DfuSansIo,
    buffer: Vec<u8>,
    progress: Option<Box<dyn FnMut(usize)>>,
}

impl<IO, E> DfuSync<IO, E>
where
    IO: DfuIo<Read = usize, Write = usize, Reset = (), Error = E>,
    E: From<std::io::Error> + From<Error>,
{
    /// Create a new instance of a generic synchronous implementation of DFU.
    pub fn new(io: IO) -> Self {
        let transfer_size = io.functional_descriptor().transfer_size as usize;
        let descriptor = *io.functional_descriptor();

        Self {
            io,
            dfu: DfuSansIo::new(descriptor),
            buffer: vec![0x00; transfer_size],
            progress: None,
        }
    }

    /// Override the address onto which the firmware is downloaded.
    ///
    /// This address is only used if the device uses the DfuSe protocol.
    pub fn override_address(&mut self, address: u32) -> &mut Self {
        self.dfu.set_address(address);
        self
    }

    /// Use this closure to show progress.
    pub fn with_progress(&mut self, progress: impl FnMut(usize) + 'static) -> &mut Self {
        self.progress = Some(Box::new(progress));
        self
    }

    /// Consume the object and return its [`DfuIo`]
    pub fn into_inner(self) -> IO {
        self.io
    }
}

impl<IO, E> DfuSync<IO, E>
where
    IO: DfuIo<Read = usize, Write = usize, Reset = (), Error = E>,
    E: From<std::io::Error> + From<Error>,
{
    /// Download a firmware into the device from a slice.
    ///
    /// Returns `Some(Self)` if the device stayed on the bus (manifestation tolerant, no USB reset
    /// occurred) or `None` if a USB reset was performed.
    pub fn download_from_slice(self, slice: &[u8]) -> Result<Option<Self>, IO::Error> {
        let length = slice.len();
        let cursor = Cursor::new(slice);
        self.download(
            cursor,
            u32::try_from(length).map_err(|_| Error::OutOfCapabilities)?,
        )
    }

    /// Download a firmware into the device from a reader.
    ///
    /// Returns `Some(Self)` if the device stayed on the bus (manifestation tolerant, no USB reset
    /// occurred) or `None` if a USB reset was performed.
    pub fn download<R: std::io::Read>(
        mut self,
        reader: R,
        length: u32,
    ) -> Result<Option<Self>, IO::Error> {
        let transfer_size = self.io.functional_descriptor().transfer_size as usize;
        let mut reader = Buffer::new(transfer_size, reader);
        let buffer = reader.fill_buf()?;
        if buffer.is_empty() {
            return Ok(Some(self));
        }

        let cmd = self.dfu.download(self.io.protocol(), length)?;
        let (cmd, mut control) = cmd.get_status(&mut self.buffer);
        let n = control.execute(&self.io)?;
        let (cmd, control) = cmd.chain(&self.buffer[..n])?;
        if let Some(control) = control {
            control.execute(&self.io)?;
        }
        let (cmd, mut control) = cmd.get_status(&mut self.buffer);
        let n = control.execute(&self.io)?;
        let mut download_loop = cmd.chain(&self.buffer[..n])??;

        loop {
            download_loop = match download_loop.next() {
                download::Step::Break => break Ok(Some(self)),
                download::Step::Erase(cmd) => {
                    let (cmd, control) = cmd.erase()?;
                    control.execute(&self.io)?;
                    wait_for_state(cmd, &self.io, &mut self.buffer)?
                }
                download::Step::SetAddress(cmd) => {
                    let (cmd, control) = cmd.set_address();
                    control.execute(&self.io)?;
                    wait_for_state(cmd, &self.io, &mut self.buffer)?
                }
                download::Step::DownloadChunk(cmd) => {
                    let chunk = reader.fill_buf()?;
                    let (cmd, control) = cmd.download(chunk)?;
                    let n = control.execute(&self.io)?;
                    reader.consume(n);
                    if let Some(progress) = self.progress.as_mut() {
                        progress(n);
                    }
                    wait_for_state(cmd, &self.io, &mut self.buffer)?
                }
                download::Step::UsbReset => {
                    log::trace!("Device reset");
                    self.io.usb_reset()?;
                    break Ok(None);
                }
            }
        }
    }

    /// Download a firmware into the device.
    ///
    /// The length is inferred from the reader. Returns `Some(Self)` if the device stayed on the
    /// bus (manifestation tolerant, no USB reset occurred) or `None` if a USB reset was performed.
    pub fn download_all<R: std::io::Read + std::io::Seek>(
        self,
        mut reader: R,
    ) -> Result<Option<Self>, IO::Error> {
        let length = u32::try_from(reader.seek(std::io::SeekFrom::End(0))?)
            .map_err(|_| Error::MaximumTransferSizeExceeded)?;
        reader.seek(std::io::SeekFrom::Start(0))?;
        self.download(reader, length)
    }

    /// Upload firmware from the device into a writer.
    ///
    /// For standard DFU, pass `u32::MAX` for `length` to read until the device signals
    /// end-of-upload. For DfuSe, pass the exact number of bytes to read.
    pub fn upload<W: std::io::Write>(
        &mut self,
        mut writer: W,
        length: u32,
    ) -> Result<(), IO::Error> {
        let cmd = self.dfu.upload(self.io.protocol(), length)?;
        let (cmd, mut control) = cmd.get_status(&mut self.buffer);
        let n = control.execute(&self.io)?;
        let (cmd, control) = cmd.chain(&self.buffer[..n])?;
        if let Some(control) = control {
            control.execute(&self.io)?;
        }
        let (cmd, mut control) = cmd.get_status(&mut self.buffer);
        let n = control.execute(&self.io)?;
        let mut upload_loop = cmd.chain(&self.buffer[..n])??;

        loop {
            upload_loop = match upload_loop.next() {
                upload::Step::Break => break,
                upload::Step::SetAddress(cmd) => {
                    let (wait, control) = cmd.set_address();
                    control.execute(&self.io)?;
                    wait_for_state(wait, &self.io, &mut self.buffer)?
                }
                upload::Step::UploadChunk(cmd) => {
                    let (recv, mut control) = cmd.upload(&mut self.buffer);
                    let n = control.execute(&self.io)?;
                    writer.write_all(&self.buffer[..n])?;
                    if let Some(progress) = self.progress.as_mut() {
                        progress(n);
                    }
                    recv.chain(n)?
                }
            };
        }

        Ok(())
    }

    /// Upload the entire firmware from the device into a writer.
    ///
    /// For DfuSe devices, the upload length is derived from the memory layout. For standard DFU
    /// devices, upload continues until the device signals end-of-upload.
    pub fn upload_all<W: std::io::Write>(&mut self, writer: W) -> Result<(), IO::Error> {
        let length = match self.io.protocol() {
            DfuProtocol::Dfu => u32::MAX,
            DfuProtocol::Dfuse { memory_layout, .. } => memory_layout.as_ref().iter().sum(),
        };
        self.upload(writer, length)
    }

    /// Send a Detach request to the device
    pub fn detach(&self) -> Result<(), IO::Error> {
        self.dfu.detach().execute(&self.io)?;
        Ok(())
    }

    /// Reset the USB device
    pub fn usb_reset(self) -> Result<IO::Reset, IO::Error> {
        self.io.usb_reset()
    }

    /// Returns whether the device will detach if requested
    pub fn will_detach(&self) -> bool {
        self.io.functional_descriptor().will_detach
    }

    /// Returns whether the device is manifestation tolerant
    pub fn manifestation_tolerant(&self) -> bool {
        self.io.functional_descriptor().manifestation_tolerant
    }
}
