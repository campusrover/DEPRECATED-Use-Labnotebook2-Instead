def list_directories(path, level = 0)
  indentation = ' ' * (level * 4) + '*'
  Dir.glob("#{path}/*").sort.each do |entry|
    if File.directory?(entry)
      dir_name = File.basename(entry)
      next if dir_name == "images"
      readme_path = "#{entry}/README.md"
      puts "#{indentation} [#{dir_name}](#{readme_path})"
      list_directories(entry, level + 1)
    elsif File.extname(entry) == ".md"
      md_file = File.basename(entry)
      puts "#{indentation} [#{md_file}](#{entry})"
    end
  end
end

root_directory = '.' # Change this to the desired directory path
list_directories(root_directory)
