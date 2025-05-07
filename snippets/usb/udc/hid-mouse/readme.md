# Sample Path

```
{workspace}/zephyr/samples/subsys/usb/hid-mouse
```

- please refer to the `README.rst` document for details.

# Build Cmd

As hpm6750evk2 for example:

```
west build -p always -b hpm6750evk2 -S hid-mouse samples/subsys/usb/hid-mouse -T sample.usb_device_next.hid-mouse
```