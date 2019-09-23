# mavlink-fast-switch

**mavlink-fast-switch** is a **select**-based **MAVLink** switch. It duplicates MAVLink messages from one endpoint to other endpoint.

## Installation

Requirments:

- libcyaml

```console
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install
```

## Configuration

Create a copy of the `example.yaml` from `/etc/mavlink-fast-switch/`, save it in the same directory and edit, according to your requirements.

## Start the application

Run `sudo systemctl start mavlink-fast-switch@<configuration>` to start the tool, where **configuration** is the configuration file name (without the file extension).

Run `sudo systemctl enable mavlink-fast-switch@<configuration>` to start the tool, where **configuration** is the configuration file name (without the file extension).
