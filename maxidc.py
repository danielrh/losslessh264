sizes = [] # size
sizes_by_file = {}

for fnam in ["0", "1", "2"]:
    sizes_by_file[fnam] = 0
    with open(fnam) as f:
        for i, lin in enumerate(f.readlines()[3:]):
            if not lin:
                continue
            if "TOTAL" in lin:
                break
            cur = int(lin.split()[2])
            if i >= len(sizes):
                sizes.append(cur)
            else:
                if cur < sizes[i]:
                    sizes[i] = cur
            sizes_by_file[fnam] += cur

print sum(sizes)
print sizes_by_file
