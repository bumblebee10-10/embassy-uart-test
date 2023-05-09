# Embassy training repo


1. watchdog seems to act very fast, much quicker than 3 seconds
2. i can't get uart to read any byte
3. uncommenting line 38 reduces binary by half, and it ceases to work. Why?
4. how can i get rid of ![no_std] vscode error? "can't find crate for `test`"

```
    "rust-analyzer.cargo.target": "thumbv6m-none-eabi",

    "rust-analyzer.check.extraArgs": [
        "--target",
        "thumbv6m-none-eabi"
    ],

    "rust-analyzer.check.noDefaultFeatures": true,
    "rust-analyzer.cargo.noDefaultFeatures": true,
```
    in `.vscode/settings.json` doesn't help.