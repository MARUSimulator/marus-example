# OceanScale ROV Sim for Unity

This is a Unity 6 standalone ROV simulator built on
[MARUSimulator/marus-example](https://github.com/MARUSimulator/marus-example) and
[marus-core](https://github.com/MARUSimulator/marus-core). It packages a MuJoCo
6-DOF underwater vehicle, articulated manipulator, HDRP underwater scene, player
HUD, scripted tasks, headless physics oracle, and reproducible Linux player build.

Paid Crest assets are not required for the release scene. The default scene uses
Unity HDRP water plus the vendored MuJoCo Unity package pinned in
`Packages/manifest.json`.

## Dependencies

* Unity `6000.0.42f1`
* git and git-lfs
* ffmpeg for demo-video encoding
* an X display for the player recording step

## Setup

```bash
git clone --recurse-submodules <repo-url>
cd marus-example
git submodule update --init --recursive
```

## Verify

```bash
UNITY=/path/to/Unity/Editor/Unity

# Headless physics + manipulator oracle. Expected: ROVTEST_RESULT ALL_PASS 25/25
"$UNITY" -batchmode -nographics -quit -projectPath . \
  -executeMethod RovSimTest.RunHeadless -logFile Logs/oracle.log

# Build Linux player, rerun oracle, record and encode a demo.
UNITY="$UNITY" DISPLAY_ID=:1 tools/reproduce_demo.sh
```

The main release notes and controls are in `Assets/RovSim/README.md`.

## Documentation
* [API Documentation](https://marusimulator.github.io/marus-api-documentation/api/index.html)

## Credits & Acknowledgements

* [gRPC](https://github.com/grpc/grpc)
* [protobuf](https://github.com/protocolbuffers/protobuf)

## Contact
Please feel free to provide feedback or ask questions by creating a Github issue. For other inquiries, please email us at labust@fer.hr or visit our web pages below:
* [Laboratory for Underwater Systems and Technologies - LABUST](https://labust.fer.hr/)

* [University of Zagreb, Faculty of Electrical Engineering and Computing](https://www.fer.unizg.hr/en)

## License
This project is released under the Apache 2.0 License. Please review the [License](https://github.com/MARUSimulator/marus-example/blob/dev/LICENSE) file for more details.
