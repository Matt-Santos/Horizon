#include <WiFi.h>
#include <esp_camera.h>
#include <WebServer.h>

class CameraServer {
private:
    const char* ssid = "YOUR_SSID";    // Replace with your Wi-Fi SSID
    const char* password = "YOUR_PASS"; // Replace with your Wi-Fi password
    WebServer server;                  // Web server object

public:
    CameraServer() : server(80) {} // Initialize web server on port 80

    // Camera setup function
    void setupCamera() {
        camera_config_t config;
        config.ledc_channel = LEDC_CHANNEL_0;
        config.ledc_timer = LEDC_TIMER_0;
        config.pin_d0 = 5;
        config.pin_d1 = 18;
        config.pin_d2 = 19;
        config.pin_d3 = 21;
        config.pin_d4 = 36;
        config.pin_d5 = 39;
        config.pin_d6 = 34;
        config.pin_d7 = 35;
        config.pin_xclk = 0;
        config.pin_pclk = 22;
        config.pin_vsync = 25;
        config.pin_href = 23;
        config.pin_sscb_sda = 26;
        config.pin_sscb_scl = 27;
        config.pin_reset = -1;
        config.xclk_freq_hz = 20000000;
        config.pixel_format = PIXFORMAT_JPEG;
        
        // Initializing the camera
        if (esp_camera_init(&config) != ESP_OK) {
            Serial.println("Camera init failed!");
            return;
        }

        Serial.println("Camera setup complete");
    }

    // Wi-Fi setup function
    void setupWiFi() {
        WiFi.begin(ssid, password);
        Serial.println();
        while (WiFi.status() != WL_CONNECTED) {
            delay(1000);
            Serial.print(".");
        }
        Serial.println("Wi-Fi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    }

    // Stream handler
    void streamHandler() {
        camera_fb_t *fb = NULL;
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            return;
        }

        server.streamFile(fb->buf, "image/jpeg");
        esp_camera_fb_return(fb);
    }

    // Setup web server routes
    void setupServer() {
        server.on("/", HTTP_GET, [this]() {
            server.send(200, "text/html", "<html><body><h1>ESP32 Camera Stream</h1><img src='/stream' width='640' height='480'></body></html>");
        });

        server.on("/stream", HTTP_GET, [this]() {
            server.sendHeader("Cache-Control", "no-store");
            server.sendHeader("Pragma", "no-cache");
            server.sendHeader("Connection", "close");
            server.send(200, "multipart/x-mixed-replace; boundary=frame", "");
            while (true) {
                streamHandler();
            }
        });

        server.begin();
        Serial.println("Server started");
    }

    // Main setup
    void begin() {
        Serial.begin(115200);
        setupWiFi();
        setupCamera();
        setupServer();
    }

    // Main loop
    void loop() {
        server.handleClient();
    }
};

CameraServer cameraServer;

void setup() {
    cameraServer.begin();
}

void loop() {
    cameraServer.loop();
}