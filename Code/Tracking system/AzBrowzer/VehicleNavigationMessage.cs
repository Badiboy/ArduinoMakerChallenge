namespace AzBrowzer
{
    /// <summary>
    /// Special container class for received data
    /// </summary>
    class VehicleNavigationMessage
    {
        //Identification string of vehicle location sensor
        public string DeviceId;

        //Stub for temperature field in receiving message
        public int Temperature;

        //Field "Humidity" is interpreted as vehicle presence in the range of the DeviceId sensor
        public int Humidity;

        //URL of a web-page depicting location of sensor device => vehicle location
        public string Url;
    }
}
