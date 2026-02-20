// ================================================================================
// KIWI-001 DECODER - TERMISTORER + WATERMARK
// Enhetliga fältnamn med Arduino-noder
// ================================================================================

function decodeUplink(input) {
    var bytes = input.bytes;
    var data = {};
    var warnings = [];
    var errors = [];
    
    try {
        var i = 0;
        while (i < bytes.length - 1) {
            var channel_id = bytes[i++];
            var channel_type = bytes[i++];
            
            // ================================================================
            // ANALOG THERMISTOR VOLTAGE (Input 3 & 4)
            // ================================================================
            
            // Thermistor Input 3
            if (channel_id === 0x03 && channel_type === 0x02) {
                if (i + 1 < bytes.length) {
                    var voltage3 = (bytes[i] << 8) | bytes[i+1];
                    if (voltage3 > 0 && voltage3 < 3000) {
                        var temp3 = -31.96 * Math.log(voltage3) + 213.25;
                        data.temp1_c = parseFloat(temp3.toFixed(1));
                    }
                    i += 2;
                }
            }
            // Thermistor Input 4
            else if (channel_id === 0x04 && channel_type === 0x02) {
                if (i + 1 < bytes.length) {
                    var voltage4 = (bytes[i] << 8) | bytes[i+1];
                    if (voltage4 > 0 && voltage4 < 3000) {
                        var temp4 = -31.96 * Math.log(voltage4) + 213.25;
                        data.temp2_c = parseFloat(temp4.toFixed(1));
                    }
                    i += 2;
                }
            }
            
            // ================================================================
            // WATERMARK SENSORS (Input 5 & 6)
            // ================================================================
            
            // Watermark #1 (Input 5)
            else if (channel_id === 0x05 && channel_type === 0x04) {
                if (i + 1 < bytes.length) {
                    var freq1 = (bytes[i] << 8) | bytes[i+1];
                    if (freq1 > 10000) freq1 = freq1 / 1000.0;
                    data.wm1_raw = freq1;
                    i += 2;
                }
            }
            // Watermark #2 (Input 6)
            else if (channel_id === 0x06 && channel_type === 0x04) {
                if (i + 1 < bytes.length) {
                    var freq2 = (bytes[i] << 8) | bytes[i+1];
                    if (freq2 > 10000) freq2 = freq2 / 1000.0;
                    data.wm2_raw = freq2;
                    i += 2;
                }
            }
            
            // ================================================================
            // KIWI INBYGGDA SENSORER
            // ================================================================
            
            // Light Intensity
            else if (channel_id === 0x09 && channel_type === 0x65) {
                if (i + 1 < bytes.length) {
                    data.light_lux = (bytes[i] << 8) | bytes[i+1];
                    i += 2;
                }
            }
            // MCU Temperature
            else if (channel_id === 0x0D && channel_type === 0x73) {
                if (i < bytes.length) {
                    data.mcu_temp_c = bytes[i];
                    i += 1;
                }
            }
            // Ambient Temperature (intern SHT30)
            else if (channel_id === 0x03 && channel_type === 0x67) {
                if (i + 1 < bytes.length) {
                    var ambient_temp = (bytes[i] << 8) | bytes[i+1];
                    data.air_temp_c = ambient_temp / 10.0;
                    i += 2;
                }
            }
            else if (channel_id === 0x0B && channel_type === 0x67) {
                if (i + 1 < bytes.length) {
                    var ambient_temp2 = (bytes[i] << 8) | bytes[i+1];
                    data.air_temp_c = ambient_temp2 / 10.0;
                    i += 2;
                }
            }
            // Ambient Humidity (intern SHT30)
            else if (channel_id === 0x04 && channel_type === 0x68) {
                if (i < bytes.length) {
                    data.air_rh_pct = bytes[i] / 2.0;
                    i += 1;
                }
            }
            else if (channel_id === 0x0B && channel_type === 0x68) {
                if (i < bytes.length) {
                    data.air_rh_pct = bytes[i] / 2.0;
                    i += 1;
                }
            }
            
            // ================================================================
            // BATTERI
            // ================================================================
            
            // Battery Capacity (%)
            else if (channel_id === 0x00 && channel_type === 0xD3) {
                if (i < bytes.length) {
                    data.battery_pct = bytes[i];
                    i += 1;
                }
            }
            // Remaining Battery Days
            else if (channel_id === 0x00 && channel_type === 0xBD) {
                if (i + 1 < bytes.length) {
                    data.battery_days = (bytes[i] << 8) | bytes[i+1];
                    i += 2;
                }
            }
            
            // ================================================================
            // OKÄND DATA
            // ================================================================
            else {
                if (channel_id !== undefined && channel_type !== undefined) {
                    warnings.push("Unknown channel: 0x" + channel_id.toString(16) + 
                                 " type: 0x" + channel_type.toString(16));
                }
                i += 2;
            }
        }
        
        return {
            data: data,
            warnings: warnings,
            errors: errors
        };
    }
    catch (error) {
        return {
            data: {},
            warnings: warnings,
            errors: ["Decoder error: " + error.message]
        };
    }
}
