# AudioLibSwitcher_OpenAL

![GitHub License](https://img.shields.io/github/license/razterizer/AudioLibSwitcher_OpenAL?color=blue)
![Static Badge](https://img.shields.io/badge/linkage-header_only-yellow)
![Static Badge](https://img.shields.io/badge/3rdparty_linkage-static_/_dynamic-yellow)
![Static Badge](https://img.shields.io/badge/C%2B%2B-20-yellow)

An adapter / interface implementation of the `IAudioLibSwitcher` interface (see [AudioLibSwitcher](https://github.com/razterizer/AudioLibSwitcher) for [OpenAL-Soft](https://github.com/kcat/openal-soft).

When cloning, don't forget to use the flag `--recurse-submodules`. For example: `git clone AudioLibSwitcher_OpenAL --recurse-submodules`

When updating the submodule hash to the latest hash:
```bash
# First cd to repo root.
git submodule update --remote AudioLibSwitcher
git add AudioLibSwitcher
git commit -m "Updated the submodule to the latest changeset."
git push
```
