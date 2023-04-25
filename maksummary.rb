require "yaml"

class MakeSummary
  def process_directory(path, indent = 0)
    entries = []
    Dir.entries(path).sort.each do |entry|
      next if skip_entry?(entry)
      current_path = File.join(path, entry)
      if File.directory?(current_path)
        entries << extract_dir_info(entry, current_path, indent)
        process_directory(current_path, indent + 4)
      elsif valid_markdown_file?(entry)
        entries << extract_file_info(entry, current_path, indent)
      end
    end
    entries
  end

  def extract_dir_info(entry, path, indent)
    title = extract_title(entry, path)
    text_string = "#{" " * indent}* [#{title}](#{path}/README.md)"
    {text: text_string, indent: indent, title: title}
  end

  def extract_file_info(entry, path, indent)
    md_title = extract_title_from_markdown(entry, path)
    text_string = "#{" " * indent}* [#{md_title}](#{path})"
    {text: text_string, indent: indent, title: md_title}
  end

  def skip_entry?(entry)
    %w[. .. images README.md SUMMARY.md docs .git .gitbook .vscode .DS_Store .bookignore .gitbook.yaml].include?(entry)
  end

  def valid_markdown_file?(entry)
    entry.end_with?(".md") && entry != "SUMMARY.md" && entry != "README.md"
  end

  def extract_title(entry, current_path)
    info_file = File.join(current_path, "info.yml")
    if File.exist?(info_file)
      info = YAML.load_file(info_file)
      return info["title"] if info["title"]
    end
    entry
  end

  def extract_title_from_markdown(entry, current_path)
    md_title = File.basename(entry, ".md")
    result = "xxx"
    begin
      md_content = YAML.load_file(current_path)
    rescue Psych::SyntaxError => e
#     puts "<RESCUE> #{result} >>> #{md_title} >>> #{e.message} >> #{md_content.class}}"
      result = md_title
      md_content = nil
    end
    begin
      result = if md_content.nil? || md_content["title"].nil?
        #    puts "() #{result} >>> #{md_title} #{md_content.class}}"
        md_title
      else
        md_content["title"]
      end
    rescue StandardError => e
      # puts "**RESCUE** #{result} >>> #{md_title} >>> #{e.message} >> #{md_content.class}}"
    end
    result
  end

  def run
    @entries = []
    Dir.entries(".").sort.each do |entry|
      # Skip based on list
      next if skip_entry?(entry)
      current_path = File.join(".", entry)

      # Top level only, skip if not a directory
      next if !File.directory?(current_path)

      dir_info = extract_dir_info(entry, current_path, 0)
      lines = process_directory(entry, 1)
      @entries << dir_info.merge!(lines: lines)
    end
    @entries.each do |entry|
      puts entry[:text]
    end
  end
end

MakeSummary.new.run
