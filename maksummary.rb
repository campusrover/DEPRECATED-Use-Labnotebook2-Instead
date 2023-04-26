require "yaml"

class MakeSummary
  def process_directory(path, indent = 0)
    entries = []
    Dir.entries(path).sort.each do |entry|
      next if skip_entry?(entry)
      current_path = File.join(path, entry)
      if File.directory?(current_path)
        dir_info = extract_dir_info(entry, current_path, indent)
        lines = process_directory(current_path, indent + 1)
        @entries << dir_info.merge!(lines:)
      elsif valid_markdown_file?(entry)
        entries << generate_entry_for_file(entry, current_path, indent)
      end
    end
    entries
  end

  def extract_dir_info(entry, path, indent)
    dir_keys = extract_keys_from_directory(entry, path)
    title = dir_keys.fetch("title", entry)
    order = dir_keys.fetch("order", 100)
    text_string = "#{'    ' * indent}* [#{title}](#{path}/README.md)"
    {text: text_string, indent:, title:, order:}
  end

  def generate_entry_for_file(entry, path, indent)
    file_keys = extract_keys_from_markdown(entry, path)
    title = file_keys.fetch("title", entry)
    order = file_keys.fetch("order", 100)
    text_string = " #{'    ' * indent}* [#{title}](#{path})"
    {text: text_string, indent:, title:, order:}
  end

  def extract_file_info(_eßntry, path, indent)
    md_title = extract_title_from_markdown(entry, path)
    {text: text_string, indent:, title: md_title}
  end

  def skip_entry?(entry)
    %w[. .. images README.md SUMMARY.md docs .git .gitbook .vscode .DS_Store .bookignore .gitbook.yaml].include?(entry)
  end

  def valid_markdown_file?(entry)
    entry.end_with?(".md")
  end

  def extract_title(entry, current_path)
    info_file = File.join(current_path, "info.yml")
    if File.exist?(info_file)
      info = YAML.load_file(info_file)
      return info["title"] if info["title"]
    end
    entry
  end

  def extract_keys_from_directory(entry, current_path)
    result = {title: entry}
    info_file = File.join(current_path, "info.yml")
    if File.exist?(info_file)
      info = YAML.load_file(info_file)
      result.merge!(order: info["order"], title: info["title"])
    end
    result
  end

  def extract_keys_from_markdown(entry, current_path)
    result = {title: entry}
    begin
      md_content = YAML.load_file(current_path)
    rescue StandardError
    end
    result.merge!(md_content) unless md_content.nil? || !md_content.is_a?(Hash)
    result
  end

  def run
    @entries = []
    Dir.entries(".").each do |entry|
      puts entry.inspect
      # Skip based on list
      next if skip_entry?(entry)

      current_path = File.join(".", entry)

      # Top level only, skip if not a directory
      next if !File.directory?(current_path)

      dir_info = extract_dir_info(entry, current_path, 0)
      lines = process_directory(entry, 1)
      order = dir_info.fetch("order", 100)
      @entries << {title: dir_info["title"], order:, lines:}
    end
    puts @entries.size
    @entries.each do |entry|
      # puts entry[:text]
      # puts(entry[:lines].sort_by { |line| line[:order] }.map { |line| line[:text] })
    end
  end
end

MakeSummary.new.run
