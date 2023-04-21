# Write a ruby script which generates text output as follows:
# Go through the current directory, recursively
# Directories are shown with their names in square brackets, followed directly by a parenthesis, the current path followed by the "README.md" and a closed parenthesis
# Files with a `.md` extension are shown with their names in square brackets and their full paths in parentheses.
# Directories named "images" or “docs” are skipped.
# Files named SUMMARY.md and READNE.md are skipped
# Each line is displayed with an indentation followed by a star and a space. The indentation starts at zero and goes up by four each level of the directory

require 'find'

def process_directory(path, depth = 0)
  indent = ' ' * depth * 4
  Dir.glob("#{path}/*").sort.each do |entry|
    next if entry.end_with?('README.md') || entry.end_with?('SUMMARY.md')
    if File.directory?(entry)
      next if entry.end_with?('images') || entry.end_with?('docs')
      puts "#{indent}* [#{File.basename(entry)}](#{entry}/README.md)"
      process_directory(entry, depth + 1)
    elsif entry.end_with?('.md')
      puts "#{indent}* [#{File.basename(entry, '.md')}](#{entry})"
    end
  end
end

process_directory('.')
