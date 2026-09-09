# Bundled libcreate

This directory contains a source snapshot of
[AutonomyLab/libcreate](https://github.com/AutonomyLab/libcreate), based on commit
`116be443e7970de1574b5dc5f91e414828854c08`, with the local ROS 2 Lyrical compatibility
changes applied. It is an ordinary directory in the `create_robot_lyrical`
repository, not a Git submodule or a separate checkout.

The upstream BSD license is retained in [LICENSE](LICENSE), along with the
copyright notices in the source files. The upstream Git metadata and standalone
GitHub Actions workflow are not included in this snapshot.

Local changes include:

- A CMake 3.20 minimum and a C++17 default for the standalone build and tests.
- Boost config-mode discovery and the `Boost::headers` / `Threads::Threads`
  targets, including the installed dependency configuration.
- Boost.Asio `io_context` / `restart()` APIs and a `steady_timer` for the
  existing 50 ms serial query recovery interval.
- Build documentation for the containing Lyrical repository.

The package name remains `libcreate`, and its exported library target remains
`create`. Colcon discovers this directory through its `package.xml` and orders
the driver build after it. Keep the library's CMake project, tests, examples,
and package manifest intact when updating it.

To incorporate future upstream fixes, compare against the revision above and
apply or merge the relevant changes into this directory, preserving the local
Lyrical adaptations. Record any new upstream baseline here.
