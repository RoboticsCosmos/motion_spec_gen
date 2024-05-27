# motion-spec-gen

## Steps

1. To generate the IR
    ```bash
    [src] $ python3 motion_spec_gen/runner.py -m <motion_spec_folder> -o <output_ir>
    ```
2. To generate code
   ```bash
   stst -s "<>" -t <folder> <name>.application <ir_name> > <output_file>
   ```