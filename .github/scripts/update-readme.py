import os
import json
import yaml

# Define the base folder and README file path
base_folder = 'releases'
readme_path = os.path.join(base_folder, 'README.md')


# Function to read data from a YAML file
def read_data(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)
    return {}

# Function to update the README file
def update_readme(folders_data):
    # Define the desired order of columns
    desired_order = ['Description', 'Version', 'Language', 'Creator']

    # Identify all unique keys to ensure all columns are included
    all_keys = set()
    for data in folders_data.values():
        all_keys.update(data.keys())
    all_keys = sorted(all_keys)

    # Ensure desired_order keys come first and add any remaining keys
    ordered_keys = [key for key in desired_order if key in all_keys] + [key for key in all_keys if key not in desired_order]

    # Create new table headers and dividers
    headers = ['Folder Name'] + ordered_keys
    header_line = '| ' + ' | '.join(headers) + ' |\n'
    divider_line = '| ' + ' | '.join(['-' * len(header) for header in headers]) + ' |\n'

    new_table = [header_line, divider_line]

    # Create table rows
    sorted_folders = sorted(folders_data.keys())
    for folder in sorted_folders:
        row = [folder] + [str(folders_data[folder].get(key, '')) for key in ordered_keys]
        new_table.append('| ' + ' | '.join(row) + ' |\n')

    # Write the new content to the README.md file
    with open(readme_path, 'w') as readme_file:
        readme_file.write('# Releases  \n')
        readme_file.writelines(new_table)

def main():
    folders_data = {}
    for folder in os.listdir(base_folder):
        folder_path = os.path.join(base_folder, folder)
        if os.path.isdir(folder_path):
            data = {}
            for file in os.listdir(folder_path):
                file_path = os.path.join(folder_path, file)
                if file.endswith('.yaml') or file.endswith('.yml'):
                    file_data = read_data(file_path)
                    data.update(file_data)
                    break
            folders_data[folder] = data

    update_readme(folders_data)

if __name__ == "__main__":
    main()
