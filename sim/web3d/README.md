# Lite3 Web MuJoCo viewer

This directory adapts the Apache-2.0 Web MuJoCo viewer from
`syswonder/robot-agilex-ranger_with_piper_mujoco` at commit
`1764366ce304d88bf8024e53baf99575e6a7d521`. It retains the upstream Three.js,
`mujoco-js`, modular environment loader, WebGL renderer, and orbit camera.

`src/utils/Lite3StateClient.js` is the integration layer added here. It mirrors
qpos/qvel from `/api/state`, which is served by the native Lite3 MuJoCo runtime.
The browser never advances a second physics simulation, so ROS, sensors,
navigation, and the displayed pose share one source of truth.

The bundled environment is the furnished SceneSmith House 187 from
`nepfaff/scenesmith-example-scenes` (`House/scene_187.tar`). The native runtime
and browser both use its MJCF, OBJ meshes, PNG textures, and `spawn.json`.
Provenance and Apache-2.0 notices are in `third_party/` and the environment's
`NOTICE.md`.

Install the pinned browser dependencies and start the complete stack:

```bash
./sim/manage.sh web3d-install
./start.sh --mode simulation --viewer web
```

From Windows, forward the remote loopback-only HTTP server and open the page:

```powershell
ssh -L 18080:127.0.0.1:8080 codex
```

```text
http://127.0.0.1:18080/
```

Use the mouse to orbit, pan, and zoom. The operator panel reports the selected
room, simulation time, running/paused state, controller mode, and command.
