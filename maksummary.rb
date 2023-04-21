def list_directories(path, level = 0)
  indentation = ' ' * 4* level + '*'
  Dir.glob("#{path}/*").sort.each do |entry|
    if File.directory?(entry)
      dir_name = File.basename(entry)
      readme_path = "#{entry}/README.md"
      puts "#{indentation} [#{dir_name}](#{readme_path})"
      list_directories(entry, level + 1)
    end
  end
end

root_directory = '.' # Change this to the desired directory path
list_directories(root_directory)