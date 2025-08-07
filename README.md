# opensurgbot_pipeline

Submodule of [opensurgbot](https://github.com/lgabp1/opensurgbot).
python library to drive the da Vinci Interface and STM32CubeIDE example integration.

## Requirements

* [python 3.7+](https://www.python.org/)
* [my da Vinci viz library](https://github.com/lgabp1/kinevizu). It is acessible as a submodule, so one can clone this repository using `git clone ---recursive https://github.com/lgabp1/opensurgbot_pipeline.git`

## Usage

Run `./main.py` using python. The program will wait for a valid COM serial device to appear, and will then connect to it. The use is then able to control via the GUI.

## Integration

Please refer to [protocol_description.md](./protocol_description.md) for more details on the protocol used.

Please also find in [STM32CubeIDE_projects/](./STM32CubeIDE_projects/) an example project made for the DM-MC02 board. It is based on the [DM-MC02_drivers](https://github.com/lgabp1/DM-MC02_drivers) drivers, as well as my [zdt_stm32_driver](https://github.com/lgabp1/zdt_stm32_driver) library.

## Licence

Not yet decided, as this repository shares a STM32CubeIDE project.