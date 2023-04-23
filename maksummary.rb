require "yaml"

class MakeSummary
  def process_directory(path, indent = 0)
    entries = []
    Dir.entries(path).sort.each do |entry|
      next if skip_entry?(entry)

      current_path = File.join(path, entry)
      if File.directory?(current_path)
        title = extract_title(entry, current_path)
        text_string = "#{" " * indent}* [#{title}](#{current_path}/README.md)"
        entries << {text: text_string, indent: indent, title: title}
        process_directory(current_path, indent + 4)
      elsif valid_markdown_file?(entry)
        md_title = extract_title_from_markdown(entry, current_path)
        text_string = "#{" " * indent}* [#{md_title}](#{current_path})"
        entries << {text: text_string, indent: indent, title: title}
      end
    end
    entries
  end

  def skip_entry?(entry)
    %w[. .. images docs .git .gitbook .vscode].include?(entry)
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
    rescue e
      #    puts "<RESCUE> #{result} >>> #{md_title} >>> #{e.message} >> #{md_content.class}}"
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
    rescue e
      # puts "**RESCUE** #{result} >>> #{md_title} >>> #{e.message} >> #{md_content.class}}"
    end
    result
  end

  # entries will look like this:
  # [{:text=>"* [Introduction](./README.md)", :indent=>0, :title=>"Introduction", :lines
  def run
    @entries = []
    Dir.entries(".").sort.each do |entry|
      next if skip_entry?(entry)
      current_path = File.join(".", entry)
      title = extract_title(entry, current_path)
      title_line = "* [#{title}](#{current_path}/README.md)"
      lines = process_directory(entry, 1)
      @entries << {text: title_line, indent: 0, title: title, lines: lines}
    end
    @entries.each do |entry|
      puts entry[:text]
    end
  end
end

MakeSummary.new.run
