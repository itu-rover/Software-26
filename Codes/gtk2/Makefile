CC = gcc
CFLAGS = -Wall -Wextra -std=c99 `pkg-config --cflags gtk+-3.0 gstreamer-1.0`
LIBS = `pkg-config --libs gtk+-3.0 gstreamer-1.0`

SOURCES = main.c ui_setup.c ui_callbacks.c systemd_service_manager.c pipeline_factory.c
OBJECTS = $(SOURCES:.c=.o)
TARGET = camera_viewer

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

install-deps:
	sudo apt-get update
	sudo apt-get install libgtk-3-dev libgstreamer1.0-dev pkg-config

.PHONY: all clean install-deps 