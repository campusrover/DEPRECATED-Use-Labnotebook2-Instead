# Write a ruby script which generates text output as follows:
# Go through the current directory, recursively
# Each directory contains a file called contains a YAML file called info.yml
# The info.yml file contains a key called "title" which contains the title of the directory
# If the file is not present then the directory name is used
# md files are also yaml files and will contain a title key which would be the title of the file. 
# if the title key is not present then the filename is used.
# Directories are shown with their title in square brackets, followed directly by a parenthesis, the current path followed by the "README.md" and a closed parenthesis
# Files with a `.md` extension are shown with their names in square brackets and their full paths in parentheses.
# Directories named "images" or “docs” or ".git" are skipped.
# Files named SUMMARY.md and README.md are skipped
# Each line is displayed with an indentation followed by a star and a space. The indentation starts at zero and goes up by four each level of the directory
require 'yaml'

# Refactor this code to make it more readable

# def process_directory(path, indent = 0)
#   Dir.entries(path).each do |entry|
#     next if entry == '.' || entry == '..' || entry == 'images' || entry == 'docs' || entry == '.git'

#     current_path = File.join(path, entry)
#     if File.directory?(current_path)
#       title = entry
#       info_file = File.join(current_path, 'info.yml')
#       if File.exist?(info_file)
#         info = YAML.load_file(info_file)
#         title = info['title'] if info['title']
#       end
#       puts "#{' ' * indent}* [#{title}](#{current_path}/README.md)"
#       process_directory(current_path, indent + 4)
#     elsif entry.end_with?('.md') && entry != 'SUMMARY.md' && entry != 'README.md'
#       md_title = File.basename(entry, '.md')
#       md_content = YAML.load_file(current_path)
#       md_title = md_content['title'] if md_content['title']
#       puts "#{' ' * indent}* [#{md_title}](#{current_path})"
#     end
#   end
# end

# process_directory('.')


require 'yaml'

def process_directory(path, indent = 0)
  Dir.entries(path).each do |entry|
    next if skip_entry?(entry)

    current_path = File.join(path, entry)
    if File.directory?(current_path)
      title = extract_title(entry, current_path)
      puts "#{' ' * indent}* [#{title}](#{current_path}/README.md)"
      process_directory(current_path, indent + 4)
    elsif valid_markdown_file?(entry)
      md_title = extract_title_from_markdown(entry, current_path)
      puts "#{' ' * indent}* [#{md_title}](#{current_path})"
    end
  end
end

def skip_entry?(entry)
  %w[. .. images docs .git].include?(entry)
end

def valid_markdown_file?(entry)
  entry.end_with?('.md') && entry != 'SUMMARY.md' && entry != 'README.md'
end

def extract_title(entry, current_path)
  info_file = File.join(current_path, 'info.yml')
  if File.exist?(info_file)
    info = YAML.load_file(info_file)
    return info['title'] if info['title']
  end
  entry
end

def extract_title_from_markdown(entry, current_path)
  md_title = File.basename(entry, '.md')
  result = "xxx"
  begin
    md_content = YAML.load_file(current_path)
  rescue Exception => e
    puts "<RESCUE> #{result} >>> #{md_title} >>> #{e.message} >> #{md_content.class}}"
    result = md_title
    md_content = nil
  end
  begin
    if md_content.nil? || md_content['title'].nil?
      puts "() #{result} >>> #{md_title} #{md_content.class}}"
      result = md_title
    end
  rescue Exception => e
    puts "**RESCUE** #{result} >>> #{md_title} >>> #{e.message} >> #{md_content.class}}"
  end
  result
end

process_directory('.')
