# Write a ruby script which generates text output as follows:
# Go through the current directory, recursively
# Each directory contains a file called contains a YAML file called info.yml
# The info.yml file contains a key called "title" which contains the title of the directory
# If the file is not present then the directory name is used
# Directories are shown with their title in square brackets, followed directly by a parenthesis, the current path followed by the "README.md" and a closed parenthesis
# Files with a `.md` extension are shown with their names in square brackets and their full paths in parentheses.
# Directories named "images" or “docs” or ".git" are skipped.
# Files named SUMMARY.md and READNE.md are skipped
# Files and directories starting with a dot are skipped
# Each line is displayed with an indentation followed by a star and a space. The indentation starts at zero and goes up by four each level of the directory

require 'yaml'

def process_directory(path, indent = 0)
  Dir.entries(path).each do |entry|
    next if entry.start_with?('.') || ['images', 'docs', '.git'].include?(entry)

    full_path = File.join(path, entry)

    if File.directory?(full_path)
      info_file = File.join(full_path, 'info.yml')
      title = entry

      if File.exist?(info_file)
        info = YAML.load_file(info_file)
        title = info['title'] if info.key?('title')
      end

      puts "#{' ' * indent}* [#{title}](#{full_path}/README.md)"
      process_directory(full_path, indent + 4)
    elsif entry.end_with?('.md') && entry != 'SUMMARY.md' && entry != 'README.md'
      puts "#{' ' * indent}* [#{entry.gsub('.md', '')}](#{full_path})"
    end
  end
end

process_directory('.')
