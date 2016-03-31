using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Input;
using Microsoft.Maker.RemoteWiring;
using Microsoft.Maker.Serial;
using Windows.UI.Xaml.Media;
using Windows.UI;
using Windows.UI.Xaml.Shapes;
 
namespace test_project
{
    public sealed partial class MainPage : Page
    {
        private SolidColorBrush _defaultBrush;
        private SolidColorBrush _pressedBrush;

        public RemoteDevice arduino;
        private BluetoothSerial bluetooth;

        const byte upPin = 7;
        const byte downPin = 9;
        const byte leftPin = 11;
        const byte rightPin = 13;
        public MainPage()
        {
            _defaultBrush = new SolidColorBrush(Colors.Black);
            _pressedBrush = new SolidColorBrush(Colors.Blue);

            this.InitializeComponent();

            connect();
        }

        private void connect()
        {
            bluetooth = new BluetoothSerial("HC-06");
            arduino = new RemoteDevice(bluetooth);
            arduino.DeviceReady += arduinoDeviceReady;
            bluetooth.begin(9600, SerialConfig.SERIAL_8N1);
        }

        private void arduinoDeviceReady()
        {
            arduino.pinMode(upPin, PinMode.OUTPUT);
            arduino.pinMode(leftPin, PinMode.OUTPUT);
            arduino.pinMode(rightPin, PinMode.OUTPUT);
            arduino.pinMode(downPin, PinMode.OUTPUT);
        }

        //Buttons
        private void downRectanglePressed(object sender, PointerRoutedEventArgs e)
        {
            arduino.digitalWrite(downPin, PinState.HIGH);
            ChangeColor(polyDown, _pressedBrush);

        }

        private void downRectangleReleased(object sender, PointerRoutedEventArgs e)
        {
            arduino.digitalWrite(downPin, PinState.LOW);

            allPointerReleased(sender, e);
        }

        private void leftRectanglePressed(object sender, PointerRoutedEventArgs e)
        {
            arduino.digitalWrite(leftPin, PinState.HIGH);
            ChangeColor(polyLeft, _pressedBrush);
        }

        private void leftRectangleReleased(object sender, PointerRoutedEventArgs e)
        {
            arduino.digitalWrite(leftPin, PinState.LOW);

            allPointerReleased(sender, e);
        }

        private void rightRectanglePressed(object sender, PointerRoutedEventArgs e)
        {
            arduino.digitalWrite(rightPin, PinState.HIGH);
            ChangeColor(polyRight, _pressedBrush);

        }

        private void rightRectangleReleased(object sender, PointerRoutedEventArgs e)
        {
            arduino.digitalWrite(rightPin, PinState.LOW);

            allPointerReleased(sender, e);
        }

        private void upRectanglePressed(object sender, PointerRoutedEventArgs e)
        {
            arduino.digitalWrite(upPin, PinState.HIGH);
            ChangeColor(polyUp, _pressedBrush);
        }

        private void upRectangleReleased(object sender, PointerRoutedEventArgs e)
        {
            arduino.digitalWrite(upPin, PinState.LOW);

            allPointerReleased(sender, e);
        }

        private void allPointerReleased(object sender, PointerRoutedEventArgs e)
        {
            ChangeColor(polyDown, _defaultBrush);
            ChangeColor(polyUp, _defaultBrush);
            ChangeColor(polyLeft, _defaultBrush);
            ChangeColor(polyRight, _defaultBrush);

            arduino.digitalWrite(upPin, PinState.LOW);
            arduino.digitalWrite(leftPin, PinState.LOW);
            arduino.digitalWrite(rightPin, PinState.LOW);
            arduino.digitalWrite(downPin, PinState.LOW);
        }

        private void ChangeColor(Polygon poly, SolidColorBrush brush)
        {
            poly.Fill = brush;
            poly.Stroke = brush;
        }
    }
}
