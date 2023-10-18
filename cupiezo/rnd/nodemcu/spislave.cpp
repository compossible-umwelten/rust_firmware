#include "SPISlave.h"

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  SPISlave.onData([](uint8_t *data, size_t len) {
    String message = String((char *)data);
    (void)len;
    if (message.equals("Hello Slave!")) {
      SPISlave.setData("Hello Master!");
    } else if (message.equals("Are you alive?")) {
      char answer[33];
      sprintf(answer, "Alive for %lu seconds!", millis() / 1000);
      SPISlave.setData(answer);
    } else {
      SPISlave.setData("Say what?");
    }
    Serial.printf("Question: %s\n", (char *)data);
  });

  SPISlave.onDataSent([]() { Serial.println("Answer Sent"); });

  SPISlave.onStatus([](uint32_t data) {
    Serial.printf("Status: %u\n", data);
    SPISlave.setStatus(millis());
  });

  SPISlave.onStatusSent([]() { Serial.println("Status Sent"); });

  SPISlave.begin();

  SPISlave.setStatus(millis());

  SPISlave.setData("Ask me a question!");
}

void loop() {}