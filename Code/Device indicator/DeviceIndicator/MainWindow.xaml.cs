using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Windows;
using System.Windows.Threading;
using InTheHand.Net.Sockets;

namespace DeviceIndicator
{
    public partial class MainWindow : Window
    {
        // Vehicle name
        private const string DeviceName = "HC-06";

        private BackgroundWorker bg1;
        private BackgroundWorker bg2;

        DispatcherTimer timerBluetooth = new DispatcherTimer();
        DispatcherTimer timerSend = new DispatcherTimer();

        private bool _active = false;

        public MainWindow()
        {
            InitializeComponent();

            // Initialize background worker for bluetooth devices search
            bg1 = new BackgroundWorker();
            bg1.DoWork += bg_DoWork;
            bg1.RunWorkerCompleted += bg_RunWorkerCompleted;

            // Initialize background worker for sending messages
            bg2 = new BackgroundWorker();
            bg2.DoWork += bg2_DoWork;
            bg2.RunWorkerCompleted += bg2_RunWorkerCompleted;

            // Read command line arguments
            ReadArgs();
        }

        // Starting timer, that will raise sending method once in a second
        private void StartSendTimer()
        {
            timerSend.Tick += sendTimer_Tick;
            timerSend.Interval = new TimeSpan(0, 0, 0, 1);
            timerSend.Start();
        }

        // Starting timer, that will raise the method for bluetooth devices search once in a second
        private void StartBluetoothTimer()
        {
            timerBluetooth.Tick += bluetoothTimer_Tick;
            timerBluetooth.Interval = new TimeSpan(0, 0, 0, 1);
            timerBluetooth.Start();
        }

        // Run background worker, sending messages
        private void sendTimer_Tick(object sender, EventArgs e)
        {
            if (!bg2.IsBusy)
                bg2.RunWorkerAsync();
        }

        // Run background worker, searching bluetooth devices
        private void bluetoothTimer_Tick(object sender, EventArgs e)
        {
            if (!bg1.IsBusy)
                bg1.RunWorkerAsync();
        }

        void ReadArgs()
        {
            int devNum = 1;

            // Read device number form command line arguments
            string[] args = Environment.GetCommandLineArgs();
            if (args.Length >= 2)
            {
                devNum = Convert.ToInt32(args[1]);
            }

            // Initialize device client for selected device
            Send.InitDeviceClient(devNum);
        }

        // A method raised after compilition of bluetooth devices search
        void bg_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            var result = (List<Device>)e.Result;

            // Check if our vehicle was found
            var neededDevice = result.Find(d => d.DeviceName == DeviceName);

            // Update indicator due to vehicle status
            if (neededDevice != null)
            {
                _active = true;
                led.IsActive = true;
                tbMAC.Text = "Name: " + neededDevice.DeviceName;
            }
            else
            {
                _active = false;
                led.IsActive = false;
                tbMAC.Text = "";
            }
        }

        void bg_DoWork(object sender, DoWorkEventArgs e)
        {
            var devices = new List<Device>();
            var bc = new BluetoothClient();

            // Search for devices with a bluetooth adapter
            BluetoothDeviceInfo[] array = bc.DiscoverDevices();

            // Add all found devices to the list
            int count = array.Length;
            for (int i = 0; i < count; i++)
            {
                var device = new Device(array[i]);
                devices.Add(device);
            }
            e.Result = devices;
        }

        void bg2_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
        }

        void bg2_DoWork(object sender, DoWorkEventArgs e)
        {
            // Send message with vehicle status
            Send.SendMessage(_active);
        }

        private void MainWindow_OnLoaded(object sender, RoutedEventArgs e)
        {
            // After GUI loaded, start all the work simultaneously
            StartBluetoothTimer();
            StartSendTimer();
        }
    }
}
