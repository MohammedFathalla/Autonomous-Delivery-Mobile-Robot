#!/usr/bin/env python3

import roslibpy

def main():
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    if client.is_connected:
        print('Is ROS connected?', client.is_connected)
    
    client.terminate()

if __name__ == '__main__':
    main()
