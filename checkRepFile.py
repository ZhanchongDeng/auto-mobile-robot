from pathlib import Path


def check():
    # store files in a set
    unique_files = set()

    for p in Path('.').iterdir():
        if p.is_dir():
            for f in p.iterdir():
                if f.is_file() and f.name not in unique_files and f.suffix == '.m':
                    unique_files.add(f.name)
                else:
                    # show which file is repeated in which directory
                    if f.suffix == '.m':
                        print(f'{f.name} in {p} is repeated')
    return unique_files

if __name__ == '__main__':
    print(check())