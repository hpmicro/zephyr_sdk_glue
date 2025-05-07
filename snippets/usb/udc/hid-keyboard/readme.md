# Sample Path

```
{workspace}/zephyr/samples/subsys/usb/hid-keyboard
```

- please refer to the `README.rst` document for details.

# Build Cmd

As hpm6750evk2 for example:

```
west build -p always -b hpm6750evk2 -S hid-keyboard samples/subsys/usb/hid-keyboard -T sample.usbd.hid-keyboard
```