# Motor Spec

Based on [T-Motor-F1404](https://n-factory.de/T-Motor-F1404-4600KV-Ultra-Light-Motor) from n-factory.de.

---

## Product Drawing

![Product Drawing](datasheets/Product%20Drawing.JPG)

## 1. Motor Specifications

These values represent the physical and electrical properties of the two motor variants.

|Specification|KV4600|
|---|---|
|**Motor Dimensions**|$\Phi 17.9 \times 16.6$ mm|
|**Stator/Magnet Config**|9N12P|
|**Shaft Diameter**|1.5 mm|
|**Lead Wire**|24#AWG 150 mm|
|**Weight (Incl. Cable)**|9.34 g|
|**Internal Resistance**|$138 m\Omega$|
|**Rated Voltage (LiPo)**|3-4S|
|**Idle Current (10V)**|0.6 A|
|**Peak Current (60s)**|20 A|
|**Max Power (60s)**|316 W|

---

## 2. Test Report: F1404 KV4600

*Tested at ~16V (4S LiPo equivalent) with an ambient temperature of 8°C.*

|Propeller|Throttle|Thrust (g)|Voltage (V)|Current (A)|RPM|Power (W)|Efficiency (g/W)|
|---|---|---|---|---|---|---|---|
|**GF3016**|50%|184.21|15.93|5.23|26532|83.28|2.21|
|**GF3016**|55%|210.22|15.91|6.37|28299|101.26|2.08|
|**GF3016**|60%|233.36|15.88|7.64|30006|121.40|1.92|
|**GF3016**|65%|254.72|15.85|8.85|31559|140.28|1.82|
|**GF3016**|70%|268.50|15.84|10.01|32770|158.49|1.69|
|**GF3016**|75%|287.60|15.81|11.32|34340|178.95|1.61|
|**GF3016**|80%|300.59|15.78|12.50|35813|197.32|1.52|
|**GF3016**|85%|316.02|15.75|13.70|37190|215.83|1.46|
|**GF3016**|90%|330.89|15.71|14.98|38436|235.41|1.41|
|**GF3016**|95%|340.24|15.68|16.24|39376|254.54|1.34|
|**GF3016**|100%|344.73|15.64|17.54|40053|274.32|1.26|

---

## Mounting Dimensions Summary

Based on the **Product Drawing**:

* **Mounting Pattern (Bottom):** 4-M2 screws on a $\Phi 9$ mm circle.
* **Propeller Mount (Top):** 4-M2 screws on a $\Phi 5$ mm circle.
* **Total Height:** 16.6 mm.
* **Shaft Length (Exposed):** 3.0 mm.
