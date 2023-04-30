require "yaml"

class MakeSummary
  def extract_dir_info(entry, path, indent)
    dir_keys = extract_keys_from_directory(entry, path)
    title = dir_keys.fetch(:title, entry)
    order = dir_keys.fetch(:order, 100)
    text_string = "#{'    ' * indent}* [#{title}](#{path}/README.md)"
    {text: text_string, indent:, title:, order:}
  end

  def generate_entry_for_file(entry, path, indent)
    file_keys = extract_keys_from_markdown(entry, path)
    title = file_keys.fetch(:title, entry)
    order = file_keys.fetch(:order, 100)
    text_string = "#{'    ' * indent}* [#{title}](#{path})"
    {text: text_string, indent:, title:, order:}
  end

  def extract_file_info(entry, path, indent)
    md_title = extract_title_from_markdown(entry, path)
    {text: text_string, indent:, title: md_title}
  end

  def skip_entry?(entry)
    %w[. .. images SUMMARY.md docs .git .gitbook .vscode .DS_Store .bookignore .gitbook.yaml].include?(entry)
  end

  def valid_markdown_file?(entry)
    entry.end_with?(".md")
  end

  def extract_title(entry, current_path)
    info_file = File.join(current_path, "info.yml")
    if File.exist?(info_file)
      info = YAML.load_file(info_file)
      info.transform_keys!(&:to_sym)
      return info[:title] if info[:title]
    end
    entry
  end

  def extract_keys_from_directory(entry, current_path)
    result = {title: entry, order: 100}
    info_file = File.join(current_path, "info.yml")
    if File.exist?(info_file)
      info = YAML.load_file(info_file)
      info.transform_keys!(&:to_sym)

      title = info.fetch(:title, entry)
      order = info.fetch(:order, 100)
      result.merge!(order:, title:)
    end
    result
  end

  def extract_keys_from_markdown(entry, current_path)
    result = {title: entry}
    begin
      md_content = YAML.load_file(current_path)
      md_content.transform_keys!(&:to_sym)
    rescue StandardError
    end
    result.merge!(md_content) unless md_content.nil? || !md_content.is_a?(Hash)
    result
  end

  def process(current, depth)
    all_entries = []
    Dir.entries(current).each do |entry|
      next if skip_entry?(entry)
      current_path = File.join(current, entry)
      if File.directory?(current_path)
        dir_info = extract_dir_info(entry, current_path, depth)
        dir_entries = process(current_path, depth + 1)
        x = dir_info.merge!(lines: dir_entries)
        all_entries << x
      elsif valid_markdown_file?(entry)
        x = generate_entry_for_file(entry, current_path, depth)
        all_entries << x
      end
    end
    all_entries
  end

  def generate_output(all_entries)
    all_entries.sort_by { |x| x[:order] }
               .each do |entry|
      puts entry[:text]
      next unless entry[:lines]
      generate_output(entry[:lines])
    end
  end

  def run
    current_path = File.join(".")
    all_entries  = process(current_path, 0)
    generate_output(all_entries)
  end
end

MakeSummary.new.run
