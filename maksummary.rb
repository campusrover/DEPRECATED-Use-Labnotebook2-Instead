require "yaml"

class MakeSummary
  def extract_dir_info(entry, path, indent)
    dir_keys = extract_keys_from_directory(entry, path)
    title = dir_keys.fetch(:title, entry)
    order = dir_keys.fetch(:order, 100)
    clazz = "directory"
    text_string = "#{'    ' * indent}* [#{title}](#{path}/README.md)"
    {text: text_string, indent:, title:, order:, clazz:}
  end

  def generate_entry_for_file(entry, path, indent)
    stat = File::Stat.new(path)
    mdate = stat.mtime
    file_keys = extract_keys_from_markdown(entry, path)
    title = file_keys.fetch(:title, entry)
    order = file_keys.fetch(:order, 100)
    type = file_keys.fetch(:type, "unspecified")
    clazz = file_keys.fetch(:class, "unspecified")
    status = file_keys.fetch(:status, "unspecified")
    text_string = "#{'    ' * indent}* [#{title}](#{path})"
    {text: text_string, indent:, title:, order:, clazz:, type:, status:, path:, mdate:}
  end

  def skip_entry?(entry)
    %w[. .. images SUMMARY.md docs .git .gitbook README.md .vscode .DS_Store .bookignore .gitbook.yaml].include?(entry)
  end

  def valid_markdown_file?(entry)
    entry.end_with?(".md")
  end

  def extract_keys_from_directory(entry, current_path)
    result = {title: entry, order: 100}
    info_file = File.join(current_path, "info.yml")
    if File.exist?(info_file)
      info = YAML.load_file(info_file)
      info.transform_keys!(&:to_sym)

      title = info.fetch(:title, entry)
      order = info.fetch(:order, 100)
      clazz = "directory"
      result.merge!(order:, title:, clazz:)
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

  def generate_special_sections(all_entries)
    puts "## Special Sections"
    puts "* New Entries"
    generate_special_section(all_entries, "new")
    puts "* Needs Review"
    generate_special_section(all_entries, "needs review")
  end

  def generate_special_section(all_entries, status_val)
    all_entries.each do |entry|
      if entry[:status] == status_val
        puts "    * [#{entry[:title]}](#{entry[:path]})"
      end
      next unless entry[:lines]
      generate_special_section(entry[:lines], status_val)
    end
  end

  def generate_output(all_entries)
    all_entries.sort_by { |x| [x[:order], x[:title]] }
               .each do |entry|
      puts entry[:text]
      next unless entry[:lines]
      generate_output(entry[:lines])
    end
  end

  def run
    current_path = File.join(".")
    all_entries  = process(current_path, 0)
    puts "## CampusRover Lab Notebook"
    generate_output(all_entries)
    generate_special_sections(all_entries)
  end
end

MakeSummary.new.run
