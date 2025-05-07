# Sample Path

```
{workspace}/zephyr/samples/subsys/usb/cdc_acm
```

- please refer to the `README.rst` document for details.

# Build Cmd

As hpm6750evk2 for example:

```
west build -p always -b hpm6750evk2 -S cdc_acm samples/subsys/usb/cdc_acm -T sample.usb_device_next.cdc-acm
```