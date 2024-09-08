void testI2C()
{
    byte error, address;
    int nDevices = 0;
    while (1)
    {
        SerialDebug.println("Scanning for I2C devices ...");
        for (address = 0x01; address < 0x7f; address++)
        {
            Wire.beginTransmission(address);
            error = Wire.endTransmission();
            if (error == 0)
            {
                SerialDebug.print("I2C device found at address 0x");
                SerialDebug.println(address, HEX);
                nDevices++;
            }
            else if (error != 2)
            {
                // SerialDebug.printf("Error %d at address 0x%02X\n", error, address);
            }
        }
        if (nDevices == 0)
        {
            SerialDebug.println("No I2C devices found");
        }
        delay(1000);
    }
}

void error(String _txt){
    Serial.println(_txt);
    while(1){
        
    }
}