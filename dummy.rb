def skip_entry?(entry)
  %w[. .. images README.md SUMMARY.md docs .git .gitbook .vscode .DS_Store .bookignore .gitbook.yaml].include?(entry)
end

x = [{x: 100}, {x: 120}, {x: 99}]

puts x.sort_by { |h| h[:x] }


# Dir.entries(".").each do |entry|
#   current_path = File.join(".", entry)
#   # Top level only, skip if not a directory
#   next if !File.directory?(current_path)
#   next if skip_entry?(entry)

#   puts entry.inspect
# end
