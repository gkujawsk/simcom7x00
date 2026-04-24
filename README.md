# SIMCOM SIM7x00 Zephyr module

Out-of-tree Zephyr module for SIMCOM SIM7500/SIM7600 modems with:

- cellular modem bring-up based on Zephyr's modern modem stack
- PPP over CMUX
- a dedicated GNSS companion driver attached through a reserved modem side
  channel

## Use as a Zephyr module

Add this directory to `ZEPHYR_EXTRA_MODULES`:

```sh
west build -b <board> <app> -- -DZEPHYR_EXTRA_MODULES=$PWD/simcom7x00
```

Example devicetree shape:

```dts
&uart1 {
	status = "okay";

	modem: modem {
		compatible = "simcom,sim7x00";
		mdm-power-gpios = <&gpio0 10 GPIO_ACTIVE_HIGH>;
		mdm-reset-gpios = <&gpio0 11 GPIO_ACTIVE_HIGH>;
		status = "okay";

		gnss {
			compatible = "simcom,sim7x00-gnss";
			status = "okay";
		};
	};
};
```
