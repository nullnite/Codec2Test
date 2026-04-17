import struct, sys, os

input_file = r"hts1a.raw"  # ← adjust this
output_file = "src/hts1a.h"

with open(input_file, "rb") as f:
    data = f.read()

samples = struct.unpack(f"<{len(data)//2}h", data)  # 16-bit signed LE

os.makedirs(os.path.dirname(output_file), exist_ok=True)
with open(output_file, "w") as f:
    f.write("#pragma once\n")
    f.write("#include <stdint.h>\n\n")
    f.write(f"// hts1a.raw — 16-bit signed PCM, 8000 Hz mono\n")
    f.write(f"// {len(samples)} samples = {len(samples)/8000:.2f}s\n\n")
    f.write(f"const int16_t hts1a_samples[] = {{\n")
    for i, s in enumerate(samples):
        if i % 16 == 0:
            f.write("    ")
        f.write(f"{s:6d},")
        if i % 16 == 15:
            f.write("\n")
    f.write("\n};\n\n")
    f.write(f"const int hts1a_num_samples = {len(samples)};\n")

print(f"Written {len(samples)} samples to {output_file}")
