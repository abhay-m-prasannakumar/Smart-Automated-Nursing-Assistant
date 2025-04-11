#define BLYNK_TEMPLATE_ID "TMPL36pjgWG6R"
#define BLYNK_TEMPLATE_NAME "Nurseasst"
#define BLYNK_AUTH_TOKEN "2uR_rp-GbiW4DAA4jdnS8fIzq-_onyVD"

#include <Arduino.h>
#include <vector>
#include <queue>
#include <map>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Galaxy";
char pass[] = "Rocky1374";

#define MOTOR1_IN1  27
#define MOTOR1_IN2  26
#define MOTOR2_IN1  25
#define MOTOR2_IN2  33
#define LED_PIN     2 // You can change this if needed
#define BUTTON_PIN 12
#define buzzerPin  32
bool buttonTriggered = false;



struct Waypoint {
    int x, y;
    bool operator==(const Waypoint& other) const {
        return x == other.x && y == other.y;
    }
    bool operator<(const Waypoint& other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }
};

struct RoomRequest {
    int room_no;
    int priority;
};

// Waypoints

std::vector<Waypoint> waypoints = {
    {0, 0}, {2, 0}, {4, 0}, // Path along X-axis
    {2, 2}, {4, 2},         // Middle waypoints
    {4, 4}                  // Final destination
};
 

std::map<int, Waypoint> room_map = {
    {101, {4, 4}},
    {102, {4, 2}},
    {103, {2, 2}},
    {0, {0, 0}} // start
};

std::vector<RoomRequest> room_queue;
bool collectingRequests = true;
unsigned long requestStartTime = 0;
const unsigned long requestDuration = 10000;

// Motor control
void moveForward() {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, LOW);
}
void turnLeft() {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, HIGH);
    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, LOW);
}
void turnRight() {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, HIGH);
}
void stopRobot() {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, LOW);
}

// ➕ LED Blink Function
void blinkLED() {
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(250);
        digitalWrite(LED_PIN, LOW);
        delay(250);
    }
}
void beepBeep() {
  // First beep
  digitalWrite(buzzerPin, HIGH);
  delay(1000); // 2 seconds
  digitalWrite(buzzerPin, LOW);
  delay(500);  // Short gap between beeps

  // Second beep
  digitalWrite(buzzerPin, HIGH);
  delay(1000); // 2 seconds
  digitalWrite(buzzerPin, LOW);
}

// A* Heuristic
int heuristic(Waypoint a, Waypoint b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

std::vector<Waypoint> getNeighbors(Waypoint current) {
    std::vector<Waypoint> neighbors;
    for (Waypoint& wp : waypoints) {
        if ((abs(wp.x - current.x) == 2 && wp.y == current.y) ||
            (abs(wp.y - current.y) == 2 && wp.x == current.x)) {
            neighbors.push_back(wp);
        }
    }
    return neighbors;
}

std::vector<Waypoint> aStar(Waypoint start, Waypoint goal) {
    std::priority_queue<std::pair<int, Waypoint>, std::vector<std::pair<int, Waypoint>>, std::greater<>> openSet;
    std::map<Waypoint, Waypoint> cameFrom;
    std::map<Waypoint, int> gScore;

    for (auto& wp : waypoints) gScore[wp] = INT_MAX;
    gScore[start] = 0;

    openSet.emplace(heuristic(start, goal), start);

    while (!openSet.empty()) {
        Waypoint current = openSet.top().second;
        openSet.pop();

        if (current == goal) {
            std::vector<Waypoint> path;
            while (!(current == start)) {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (Waypoint neighbor : getNeighbors(current)) {
            int tentativeGScore = gScore[current] + 1;
            if (tentativeGScore < gScore[neighbor]) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeGScore;
                int fScore = tentativeGScore + heuristic(neighbor, goal);
                openSet.emplace(fScore, neighbor);
            }
        }
    }
    return {};
}

void moveThroughWaypoints(std::vector<Waypoint> path) {
    int currentDirection = 0; // 0: right, 1: down, 2: left, 3: up

    for (size_t i = 1; i < path.size(); i++) {
        Waypoint prev = path[i - 1];
        Waypoint curr = path[i];

        int dx = curr.x - prev.x;
        int dy = curr.y - prev.y;

        int newDirection;
        if (dx == 2) newDirection = 0;       // Right
        else if (dx ==-2) newDirection = 2; // Left
        else if (dy == 2) newDirection = 1;  // Down
        else if (dy == -2) newDirection = 3; // Up
        else continue;

        int turn = (newDirection - currentDirection + 4) % 4;
       
            if (prev.x == 4 && prev.y == 4 && curr.x == 4 && curr.y == 2) {
            Serial.println("🔄 Turning 180° from (4,4) to (4,2)");
            turn = 2;
        }

        // 🔹 Ensure left turn at (4,2) → (2,2)
        if (prev.x == 4 && prev.y == 2 && curr.x == 2 && curr.y == 2) {
            Serial.println("🔄 Turning LEFT at (4,2) to (2,2)");
            turn = 3;
        }

        // 🔹 Ensure right turn at (2,2) → (2,0)
        if (prev.x == 2 && prev.y == 2 && curr.x == 2 && curr.y == 0) {
            Serial.println("🔄 Turning RIGHT at (2,2) to (2,0)");
            turn = 1;
        }

        // 🔹 Ensure left turn at (2,0) → (0,0)
        if (prev.x == 2 && prev.y == 0 && curr.x == 0 && curr.y == 0) {
            Serial.println("🔄 Turning LEFT at (2,0) to (0,0)");
            turn = 3;
        }


        if (turn == 1) {
            Serial.println("Turning right");
            turnRight();
            delay(500); // Adjust based on your robot’s turn time
        } else if (turn == 3) {
            Serial.println("Turning left");
            turnLeft();
            delay(500);
        } else if (turn == 2) {
            Serial.println("Turning 180");
            turnRight(); delay(800);
            turnRight(); delay(500);
        }
        Serial.printf("Turn: %d (New Direction: %d, Current Direction: %d)\n", turn, newDirection, currentDirection);

        currentDirection = newDirection;

        Serial.printf("Moving to: (%d, %d)\n", curr.x, curr.y);
        moveForward();
        delay(800);  // Adjust based on your robot’s step distance
        stopRobot();
        delay(200);
    }
}


// Blynk priorities
BLYNK_WRITE(V3) { room_queue.push_back({101, param.asInt()}); }
BLYNK_WRITE(V0) { room_queue.push_back({102, param.asInt()}); }
BLYNK_WRITE(V1) { room_queue.push_back({103, param.asInt()}); }

void setup() {
    Serial.begin(115200);
    Blynk.begin(auth, ssid, pass);
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN1, OUTPUT);
    pinMode(MOTOR2_IN2, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    pinMode(LED_PIN, OUTPUT); // ➕ Initialize LED pin
    pinMode(BUTTON_PIN, INPUT_PULLUP); // assuming pushbutton connects to GND when pressed

    digitalWrite(LED_PIN, LOW);
    
    requestStartTime = millis();
}

void loop() {
    Blynk.run();

    // Check for button press
    if (digitalRead(BUTTON_PIN) == LOW && !buttonTriggered) {
    buttonTriggered = true;
    Serial.println("Button pressed - Visiting all rooms in order...");

    Waypoint start = {0, 0};
    std::vector<int> roomOrder = {101, 102, 103};

    for (int room : roomOrder) {
        if (room_map.find(room) != room_map.end()) {
            Waypoint destination = room_map[room];
            Serial.printf("Navigating to Room %d at (%d, %d)\n", room, destination.x, destination.y);
            auto path = aStar(start, destination);
            moveThroughWaypoints(path);
            stopRobot();
            blinkLED();
            beepBeep();
            start = destination;
        }
    }

    // ➕ Return to (0, 0) after visiting all rooms
    if (!(start == Waypoint{0, 0})) {
        Serial.println("Returning to starting point (0,0)...");
        auto returnPath = aStar(start, {0, 0});
        moveThroughWaypoints(returnPath);
        stopRobot();
        blinkLED();
        beepBeep();
    }

    Serial.println("Completed room visit from button press.");
    buttonTriggered = false;
    delay(1000);  // debounce time
}


    // Handle Blynk request window
    if (millis() - requestStartTime < requestDuration) {
        collectingRequests = true;
    } else {
        collectingRequests = false;
    }

    if (!collectingRequests && !room_queue.empty()) {
        Serial.println("Processing queued requests with priority...");

        std::sort(room_queue.begin(), room_queue.end(), [](RoomRequest a, RoomRequest b) {
            return a.priority < b.priority;
        });

        Waypoint start = {0, 0};

        for (RoomRequest request : room_queue) {
            if (room_map.find(request.room_no) != room_map.end()) {
                Waypoint destination = room_map[request.room_no];
                Serial.printf("Navigating to Room %d (Priority %d) at (%d, %d)\n", request.room_no, request.priority, destination.x, destination.y);
                auto path = aStar(start, destination);
                moveThroughWaypoints(path);
                stopRobot();
                blinkLED();
                beepBeep();
                start = destination;
            }
        }
                // Return to start
        if (!(start == Waypoint{0, 0})) {
            Serial.println("Returning to starting point (0,0) after Blynk requests...");
            auto returnPath = aStar(start, {0, 0});
            moveThroughWaypoints(returnPath);
            stopRobot();
            blinkLED();
            beepBeep();
            start={0,0};//update position
        }


        room_queue.clear();
        requestStartTime = millis(); // reset
        collectingRequests = true;
    }
}