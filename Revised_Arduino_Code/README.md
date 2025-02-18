# Platform.io code for the BYU 2024 NASA BIG Ideas Challenge

## Command to run code (using the bash script for linix-based terminals and .zsh for powershell):

```bash
./upload_code.bash <ROLLER-NUM -- required> <COM PORT -- optional>
```

```Zsh
./upload_code.zsh <ROLLER-NUM -- required> <COM PORT -- optional>
```

Command to run code directly: ` pio run -t upload -e ROLLER_00004 --upload-port COM3 ` run this in the terminal, ensuring that you change `COM3` to match the correct COM port, and `ROLLER_00004` to the correct roller number you wish to flash.

> [!WARNING]
> Before making any edits, please ensure that you have created a branch using the `git branch <New Branch Name>` command.


> Running into errors? see our [Common errors document.](common_errors.md)
