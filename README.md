# motion-spec-gen

## Steps

1. To generate the IR for UC1

    ```bash
    [src] $ python3 motion_spec_gen/runner.py -m freddy_uc1 -o freddy_uc1_final
    ```

2. To generate code

   ```bash
   stst -s "<>" -t <folder> <name>.application <ir_name> > <output_file>

   stst -s "<>" -t motion_spec_gen/models/templates/ freddy/motion_spec_uc1.application motion_spec_gen/irs/freddy_uc1_final.json > motion_spec_gen/gen/freddy_uc1_generated.ncpp
   ```
