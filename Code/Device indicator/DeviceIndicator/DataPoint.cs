namespace DeviceIndicator
{
    /// <summary>
    /// A storage for sending data
    /// </summary>
    class VehicleNavigationMessage
    {
        //Identification string of vehicle location sensor
        public string DeviceId;

        //Field "Humidity" is interpreted as vehicle presence in the range of the DeviceId sensor
        public int Humidity = 0;

        //URL of a web-page depicting location of sensor device => vehicle location
        public string URL = "";

        public VehicleNavigationMessage(string devId)
        {
            DeviceId = devId;
        }
    }
}
