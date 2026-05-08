use functional_descriptor::FunctionalDescriptor;

use super::*;

const REQUEST_TYPE: u8 = 0b00100001;
const DFU_UPLOAD: u8 = 2;
const DFU_DNLOAD: u8 = 1;

/// Starting point to upload firmware from a device.
#[must_use]
pub struct Start<'dfu> {
    pub(crate) descriptor: &'dfu FunctionalDescriptor,
    pub(crate) remaining: u32,
    pub(crate) protocol: ProtocolData,
}

#[derive(Debug, Copy, Clone)]
pub(crate) struct DfuseProtocolData {
    pub address: u32,
    pub address_set: bool,
}

#[derive(Debug, Copy, Clone)]
pub(crate) enum ProtocolData {
    Dfu,
    Dfuse(DfuseProtocolData),
}

impl<'dfu> ChainedCommand for Start<'dfu> {
    type Arg = get_status::GetStatusMessage;
    type Into = Result<UploadLoop<'dfu>, Error>;

    fn chain(
        self,
        get_status::GetStatusMessage {
            status: _,
            poll_timeout: _,
            state,
            index: _,
        }: Self::Arg,
    ) -> Self::Into {
        log::trace!("Starting upload process");
        if state == State::DfuIdle {
            let block_num = match self.protocol {
                ProtocolData::Dfu => 0,
                ProtocolData::Dfuse(_) => 2,
            };
            Ok(UploadLoop {
                descriptor: self.descriptor,
                remaining: self.remaining,
                protocol: self.protocol,
                block_num,
                eof: false,
            })
        } else {
            Err(Error::InvalidState {
                got: state,
                expected: State::DfuIdle,
            })
        }
    }
}

/// Upload loop.
#[must_use]
pub struct UploadLoop<'dfu> {
    descriptor: &'dfu FunctionalDescriptor,
    protocol: ProtocolData,
    remaining: u32,
    block_num: u16,
    eof: bool,
}

impl<'dfu> UploadLoop<'dfu> {
    /// Get the next step in the upload loop.
    pub fn next(self) -> Step<'dfu> {
        if self.eof || self.remaining == 0 {
            log::trace!("Upload loop ended");
            return Step::Break;
        }

        match self.protocol {
            ProtocolData::Dfuse(d) if !d.address_set => {
                log::trace!("Upload loop: set address");
                Step::SetAddress(SetAddress {
                    descriptor: self.descriptor,
                    remaining: self.remaining,
                    protocol: d,
                    block_num: self.block_num,
                })
            }
            _ => {
                log::trace!("Upload loop: upload chunk");
                Step::UploadChunk(UploadChunk {
                    descriptor: self.descriptor,
                    remaining: self.remaining,
                    block_num: self.block_num,
                    protocol: self.protocol,
                })
            }
        }
    }
}

/// Upload step in the loop.
#[allow(missing_docs)]
pub enum Step<'dfu> {
    Break,
    SetAddress(SetAddress<'dfu>),
    UploadChunk(UploadChunk<'dfu>),
}

/// Set the address for upload (DfuSe only).
#[must_use]
pub struct SetAddress<'dfu> {
    descriptor: &'dfu FunctionalDescriptor,
    remaining: u32,
    protocol: DfuseProtocolData,
    block_num: u16,
}

impl<'dfu> SetAddress<'dfu> {
    /// Set the address for upload.
    pub fn set_address(
        self,
    ) -> (
        get_status::WaitState<UploadLoop<'dfu>>,
        UsbWriteControl<[u8; 5]>,
    ) {
        let next_protocol = ProtocolData::Dfuse(DfuseProtocolData {
            address_set: true,
            ..self.protocol
        });
        let next = get_status::WaitState::new(
            State::DfuDnbusy,
            State::DfuDnloadIdle,
            UploadLoop {
                descriptor: self.descriptor,
                remaining: self.remaining,
                protocol: next_protocol,
                block_num: self.block_num,
                eof: false,
            },
        );
        let control = UsbWriteControl::new(
            REQUEST_TYPE,
            DFU_DNLOAD,
            0,
            <[u8; 5]>::from(SetAddressCommand(self.protocol.address)),
        );
        (next, control)
    }
}

/// Upload a chunk of firmware from the device.
#[must_use]
pub struct UploadChunk<'dfu> {
    descriptor: &'dfu FunctionalDescriptor,
    remaining: u32,
    block_num: u16,
    protocol: ProtocolData,
}

impl<'dfu> UploadChunk<'dfu> {
    /// Prepare the upload request. The received firmware data will be placed in `buffer`.
    pub fn upload<'buf>(
        self,
        buffer: &'buf mut [u8],
    ) -> (UploadChunkRecv<'dfu>, UsbReadControl<'buf>) {
        let transfer_size = self.descriptor.transfer_size as usize;
        let len = buffer
            .len()
            .min(transfer_size)
            .min(self.remaining as usize);
        let control =
            UsbReadControl::new(REQUEST_TYPE, DFU_UPLOAD, self.block_num, &mut buffer[..len]);
        let recv = UploadChunkRecv {
            descriptor: self.descriptor,
            remaining: self.remaining,
            block_num: self.block_num,
            protocol: self.protocol,
            transfer_size,
        };
        (recv, control)
    }
}

/// Receives the result of a DFU_UPLOAD request.
#[must_use]
pub struct UploadChunkRecv<'dfu> {
    descriptor: &'dfu FunctionalDescriptor,
    remaining: u32,
    block_num: u16,
    protocol: ProtocolData,
    transfer_size: usize,
}

impl<'dfu> UploadChunkRecv<'dfu> {
    /// Processes the upload result. `n` is the number of bytes received.
    pub fn chain(self, n: usize) -> Result<UploadLoop<'dfu>, Error> {
        let n32 = u32::try_from(n).map_err(|_| Error::MaximumTransferSizeExceeded)?;
        Ok(UploadLoop {
            descriptor: self.descriptor,
            remaining: self.remaining.saturating_sub(n32),
            protocol: self.protocol,
            block_num: self.block_num.wrapping_add(1),
            eof: n < self.transfer_size,
        })
    }
}

#[derive(Debug, Clone, Copy)]
struct SetAddressCommand(u32);

impl From<SetAddressCommand> for [u8; 5] {
    fn from(cmd: SetAddressCommand) -> Self {
        let mut buf = [0u8; 5];
        buf[0] = 0x21;
        buf[1..].copy_from_slice(&cmd.0.to_le_bytes());
        buf
    }
}
