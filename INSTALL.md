## UI Part

#### Qt 5.7
- Download and install Qt from https://www.qt.io/download/ (at least Qt 5.7 is required)
- Open the Qt-project file LIMoSim/ui/LIMoSim.pro

#### Known issues
- On Debian distributions, if you get a build error saying "#include <Qt3DCore/...>: No such file or directory", you maybe need to install Qt3D packages:
```
$ sudo apt install qt3d5-dev
```