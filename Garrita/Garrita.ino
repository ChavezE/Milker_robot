// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------

#include <NewPing.h>

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 30 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
}

void loop() {
  int arr[20];
  int total = 0;
  for(int iA = 0; iA < 20; iA++)
  {
    arr[iA] = sonar.ping_cm();
    delay(75);
    if(arr[iA] == 0)
    {
      int count = 0;
      while(arr[iA] == 0 && count < 5)
      {
        arr[iA] = sonar.ping_cm();
        delay(75); 
        count++;
      }
    }
    total += arr[iA];
  }
  total /= 20;
  Serial.print("Distance: "); Serial.println(total);
//  delay(100);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
//  Serial.print("Ping: ");
//  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
//  Serial.println("cm");
}
