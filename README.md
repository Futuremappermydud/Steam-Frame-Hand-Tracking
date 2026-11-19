# Frame Hand Tracking

Use your hands instead of controllers on the **Steam Frame**.

Point at something with your hand, pinch your thumb and index finger together to click, and keep them pinched
to drag. Games and apps that support hand tracking also see your whole hand.

This is an early prototype. It has been tested on one headset, so expect rough edges.

## Before you install

- **You install and use this at your own risk.** It is an unofficial hobby project and comes with no warranty.
  It adds a SteamVR driver and a background service with administrator rights to your headset.
- **This is not a Valve product.** The project and the mascot it shows in the headset are not connected to,
  affiliated with or endorsed by Valve.

## Install

1. Open a terminal on the headset (Konsole in Desktop mode, or connect over SSH).
2. Paste this line and press Enter:

   ```sh
   curl -fsSL https://raw.githubusercontent.com/ZzzDevXR/Steam-Frame-Hand-Tracking/main/install.sh | sudo bash
   ```

3. Wait until it says "Frame Hand Tracking is installed". It takes about half a minute and needs internet.
4. Restart the headset.
5. Open the SteamVR dashboard, choose **Hand tracking** and press **Turn on**.
6. The headset asks if hand tracking may use the cameras. Choose **Allow**.

That's it. Put the controllers down and point with your hand.

The installer also sets up [Frame Camera Access](https://github.com/ZzzDevXR/Steam-Frame-Camera-Access), which
is what lets hand tracking see through the headset's cameras.

## Update

Run the install line again. It replaces the old version with the newest one.

## Uninstall

To remove hand tracking:

```sh
bash ~/Steam-Frame-Hand-Tracking/uninstall.sh
```

To remove hand tracking and Frame Camera Access together:

```sh
sudo bash ~/Steam-Frame-Hand-Tracking/uninstall.sh --all
```

Then restart the headset. After that you can delete the `Steam-Frame-Hand-Tracking` and
`Steam-Frame-Camera-Access` folders in your home folder.

## Known issues

- In the current version the hand ray sometimes does not show up.

## Support

If this is useful to you, you can [buy me a coffee](https://buymeacoffee.com/zzzdevxr).

## More

- How it works, options and building it yourself: [DEVELOPERS.md](DEVELOPERS.md)
- Made by [ZzzDevXR](https://github.com/ZzzDevXR), under the MIT License ([LICENSE](LICENSE)).
- The hand tracking models were made by Moshi Turner at Collabora for [Monado](https://monado.freedesktop.org).
  They and the other third-party parts keep their own terms: see [models/README.md](models/README.md) and
  [THIRD_PARTY.md](THIRD_PARTY.md).
