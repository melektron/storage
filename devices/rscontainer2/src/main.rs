fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();
    //use esp_idf_svc::hal::gpio;
    //let mypin = esp_idf_svc::hal::gpio::PinDriver::output(gpio::Gpio12);
    //esp_idf_svc::sys::esp_lcd_panel_draw_bitmap(panel, x_start, y_start, x_end, y_end, color_data)

    log::info!("Hello, world!");
    log::error!("This is an error");
}

fn add(a: i32, b: i32) -> i32 {
    a + b
}
