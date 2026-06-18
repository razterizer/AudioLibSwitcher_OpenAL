# Release notes

## 1.0.0.9

- Capture Linux Forge release preparation diagnostics as workflow artifacts.

## 1.0.0.8

- Simplify Linux Forge release publishing to the cbox workflow path.

## 1.0.0.7

- Publish target-specific Forge packages for OpenAL system requirements.
- Use system OpenAL Soft on Linux and macOS while keeping `3rdparty_OpenAL` as
  the Windows dependency.

## 1.0.0.6

- Made `AudioLibSwitcher_OpenAL.h` include the standard-library headers it uses
  directly.

## 1.0.0.5

- Capture hosted release preparation output as a workflow artifact while
  diagnosing the Windows release path.

## 1.0.0.4

- Pin the Windows release workflow to Forge 0.8.1 while validating the OpenAL
  dependency release path.

## 1.0.0.3

- Rebuild the release with Forge 0.8.1, which accepts compatible MSVC runner
  compiler-version drift.

## 1.0.0.2

- Use the corrected `3rdparty_OpenAL` cbox that preserves the
  `OpenAL_Soft/` include namespace.
- Keep automatic release publication on Windows until macOS and Linux OpenAL
  cboxes are available.

## 1.0.0.1

- Initial release.
