#!/usr/bin/env bash
bash -c "usbipd -D && sleep 5; && usbip bind --busid 1-1.3"
