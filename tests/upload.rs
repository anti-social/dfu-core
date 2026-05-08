use mock::MockIO;

mod mock;

fn setup() {
    let _ = env_logger::builder()
        .is_test(true)
        .filter_level(log::LevelFilter::Trace)
        .parse_default_env()
        .try_init();
}

fn make_firmware(size: u32) -> Vec<u8> {
    (0..size).map(|i| i as u8).collect()
}

fn test_simple_upload(mock: MockIO) {
    let size = mock.size();
    let address = mock.address();
    let firmware = make_firmware(size);

    let mut dfu = dfu_core::synchronous::DfuSync::new(mock);

    if let Some(address) = address {
        dfu.override_address(address);
    }

    let mut output = Vec::new();
    dfu.upload_all(&mut output).unwrap();

    assert_eq!(firmware, output.as_slice());
}

#[test]
fn upload_standard_dfu() {
    setup();
    let size = 128u32;
    let firmware = make_firmware(size);
    let mock = mock::MockIOBuilder::default()
        .can_upload(true)
        .upload_data(firmware)
        .build();
    test_simple_upload(mock);
}

#[test]
fn upload_dfuse() {
    setup();
    let mock_base = mock::MockIOBuilder::default()
        .can_upload(true)
        .dfuse(true)
        .build();
    let size = mock_base.size();
    let firmware = make_firmware(size);

    let mock = mock::MockIOBuilder::default()
        .can_upload(true)
        .dfuse(true)
        .upload_data(firmware)
        .build();
    test_simple_upload(mock);
}

#[test]
fn upload_dfuse_override_address() {
    setup();
    let mock_base = mock::MockIOBuilder::default()
        .can_upload(true)
        .dfuse(true)
        .build();
    let size = mock_base.size();
    let firmware = make_firmware(size);

    let mock = mock::MockIOBuilder::default()
        .can_upload(true)
        .dfuse(true)
        .address(0x08004000)
        .upload_data(firmware)
        .build();
    test_simple_upload(mock);
}

#[test]
fn upload_explicit_length() {
    setup();
    let size = 12u32; // exactly 2 full blocks (transfer_size=6)
    let firmware = make_firmware(size);
    let mock = mock::MockIOBuilder::default()
        .can_upload(true)
        .upload_data(firmware.clone())
        .build();

    let mut dfu = dfu_core::synchronous::DfuSync::new(mock);
    let mut output = Vec::new();
    dfu.upload(&mut output, size).unwrap();

    assert_eq!(firmware, output);
}

#[test]
fn upload_not_capable_returns_error() {
    setup();
    let mock = mock::MockIOBuilder::default().build(); // can_upload = false
    let mut dfu = dfu_core::synchronous::DfuSync::new(mock);
    let mut output = Vec::new();
    assert!(dfu.upload_all(&mut output).is_err());
}
