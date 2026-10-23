#include "vsim/can/dbc.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

#include "vsim/util/paths.hpp"

namespace vsim {
namespace {

// --- Tokenizer: identifiers, numbers, strings, punctuation ---------------------------------

struct Token {
  enum class Type { Word, Number, String, Punct, End } type = Type::End;
  std::string text;
  int line = 0;
};

class Lexer {
 public:
  Lexer(const std::string& text, std::string source) : text_(text), source_(std::move(source)) {}

  Token next() {
    skip_space();
    Token t;
    t.line = line_;
    if (pos_ >= text_.size()) {
      return t;
    }
    const char c = text_[pos_];
    if (c == '"') {
      ++pos_;
      std::string s;
      while (pos_ < text_.size() && text_[pos_] != '"') {
        if (text_[pos_] == '\\' && pos_ + 1 < text_.size()) {
          ++pos_;
        }
        if (text_[pos_] == '\n') {
          ++line_;
        }
        s += text_[pos_++];
      }
      ++pos_;
      t.type = Token::Type::String;
      t.text = s;
      return t;
    }
    const auto is_word = [](char ch) { return std::isalnum(static_cast<unsigned char>(ch)) || ch == '_'; };
    if (std::isdigit(static_cast<unsigned char>(c)) || ((c == '-' || c == '+' || c == '.') && pos_ + 1 < text_.size() &&
                                                         (std::isdigit(static_cast<unsigned char>(text_[pos_ + 1])) ||
                                                          text_[pos_ + 1] == '.'))) {
      std::size_t end = pos_ + 1;
      while (end < text_.size() && (std::isalnum(static_cast<unsigned char>(text_[end])) || text_[end] == '.' ||
                                    ((text_[end] == '-' || text_[end] == '+') &&
                                     (text_[end - 1] == 'e' || text_[end - 1] == 'E')))) {
        ++end;
      }
      t.type = Token::Type::Number;
      t.text = text_.substr(pos_, end - pos_);
      pos_ = end;
      return t;
    }
    if (is_word(c)) {
      std::size_t end = pos_;
      while (end < text_.size() && is_word(text_[end])) {
        ++end;
      }
      t.type = Token::Type::Word;
      t.text = text_.substr(pos_, end - pos_);
      pos_ = end;
      return t;
    }
    t.type = Token::Type::Punct;
    t.text = std::string(1, c);
    ++pos_;
    return t;
  }

  Token peek() {
    const std::size_t pos = pos_;
    const int line = line_;
    Token t = next();
    pos_ = pos;
    line_ = line;
    return t;
  }

  // Rest of the statement up to ';' (skipped).
  void skip_statement() {
    while (pos_ < text_.size() && text_[pos_] != ';') {
      if (text_[pos_] == '"') {
        next();
        continue;
      }
      if (text_[pos_] == '\n') {
        ++line_;
      }
      ++pos_;
    }
    ++pos_;
  }

  // Rest of the current line (BU_:, NS_ lists).
  std::string rest_of_line() {
    const std::size_t end = std::min(text_.find('\n', pos_), text_.size());
    std::string s = text_.substr(pos_, end - pos_);
    pos_ = end;
    return s;
  }

  [[noreturn]] void fail(const std::string& what, const Token& t) const {
    throw std::invalid_argument(source_ + ":" + std::to_string(t.line) + ": " + what + " (at '" + t.text + "')");
  }

  void skip_space() {
    while (pos_ < text_.size()) {
      const char c = text_[pos_];
      if (c == '\n') {
        ++line_;
        ++pos_;
      } else if (c == ' ' || c == '\t' || c == '\r') {
        ++pos_;
      } else if (c == '/' && pos_ + 1 < text_.size() && text_[pos_ + 1] == '/') {
        while (pos_ < text_.size() && text_[pos_] != '\n') {
          ++pos_;
        }
      } else {
        break;
      }
    }
  }

 private:
  const std::string& text_;
  std::string source_;
  std::size_t pos_ = 0;
  int line_ = 1;
};

double number(const Token& t, Lexer& lex) {
  if (t.type != Token::Type::Number) {
    lex.fail("number expected", t);
  }
  return std::stod(t.text);
}

void expect(Lexer& lex, const std::string& punct) {
  const Token t = lex.next();
  if (t.text != punct) {
    lex.fail("'" + punct + "' expected", t);
  }
}

std::uint64_t mask(int bits) { return bits >= 64 ? ~std::uint64_t{0} : (std::uint64_t{1} << bits) - 1; }

// Payload bit positions of a signal, LSB of the raw value first.
std::vector<int> bit_positions(const DbcSignal& s) {
  std::vector<int> pos(static_cast<std::size_t>(s.length));
  if (s.little_endian) {
    for (int i = 0; i < s.length; ++i) {
      pos[static_cast<std::size_t>(i)] = s.start_bit + i;
    }
  } else {
    // Motorola: start bit is the MSB; walk down within a byte, then to the
    // next byte's bit 7 (sawtooth numbering).
    int p = s.start_bit;
    for (int i = s.length - 1; i >= 0; --i) {
      pos[static_cast<std::size_t>(i)] = p;
      if (p % 8 == 0) {
        p += 15;
      } else {
        --p;
      }
    }
  }
  return pos;
}

std::string format_number(double v) {
  std::ostringstream os;
  os.precision(15);
  os << v;
  return os.str();
}

}  // namespace

int dlc_to_length(int dlc) {
  static const int table[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
  return table[std::clamp(dlc, 0, 15)];
}

int length_to_dlc(int length) {
  for (int dlc = 0; dlc < 16; ++dlc) {
    if (dlc_to_length(dlc) >= length) {
      return dlc;
    }
  }
  return 15;
}

std::uint64_t DbcSignal::encode(double physical) const {
  if (kind == Kind::Float32) {
    const float f = static_cast<float>((physical - offset) / factor);
    std::uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof bits);
    return bits;
  }
  if (kind == Kind::Float64) {
    const double d = (physical - offset) / factor;
    std::uint64_t bits = 0;
    std::memcpy(&bits, &d, sizeof bits);
    return bits;
  }
  double raw = std::round((physical - offset) / factor);
  if (!std::isfinite(raw)) {
    raw = 0.0;
  }
  // Saturate to the field (a real ECU's conversion would too, or wrap: saturating is the safe reading).
  const double lo = is_signed ? -std::ldexp(1.0, length - 1) : 0.0;
  const double hi = is_signed ? std::ldexp(1.0, length - 1) - 1.0 : std::ldexp(1.0, length) - 1.0;
  raw = std::clamp(raw, lo, hi);
  const auto value = static_cast<std::int64_t>(raw);
  return static_cast<std::uint64_t>(value) & mask(length);
}

double DbcSignal::decode(std::uint64_t raw) const {
  if (kind == Kind::Float32) {
    const auto bits = static_cast<std::uint32_t>(raw);
    float f = 0.0F;
    std::memcpy(&f, &bits, sizeof f);
    return static_cast<double>(f) * factor + offset;
  }
  if (kind == Kind::Float64) {
    double d = 0.0;
    std::memcpy(&d, &raw, sizeof d);
    return d * factor + offset;
  }
  raw &= mask(length);
  double value = static_cast<double>(raw);
  if (is_signed && length < 64 && (raw >> (length - 1)) & 1U) {
    value = static_cast<double>(static_cast<std::int64_t>(raw | ~mask(length)));
  } else if (is_signed && length == 64) {
    value = static_cast<double>(static_cast<std::int64_t>(raw));
  }
  return value * factor + offset;
}

void DbcSignal::write(std::vector<std::uint8_t>& data, double physical) const {
  const std::uint64_t raw = encode(physical);
  const std::vector<int> pos = bit_positions(*this);
  for (std::size_t i = 0; i < pos.size(); ++i) {
    const auto byte = static_cast<std::size_t>(pos[i] / 8);
    if (byte >= data.size()) {
      continue;
    }
    const auto bit = static_cast<std::uint8_t>(1U << (pos[i] % 8));
    if ((raw >> i) & 1U) {
      data[byte] = static_cast<std::uint8_t>(data[byte] | bit);
    } else {
      data[byte] = static_cast<std::uint8_t>(data[byte] & ~bit);
    }
  }
}

std::uint64_t DbcSignal::read_raw(const std::vector<std::uint8_t>& data) const {
  const std::vector<int> pos = bit_positions(*this);
  std::uint64_t raw = 0;
  for (std::size_t i = 0; i < pos.size(); ++i) {
    const auto byte = static_cast<std::size_t>(pos[i] / 8);
    if (byte < data.size() && ((data[byte] >> (pos[i] % 8)) & 1U)) {
      raw |= std::uint64_t{1} << i;
    }
  }
  return raw;
}

double DbcSignal::read(const std::vector<std::uint8_t>& data) const { return decode(read_raw(data)); }

const DbcSignal* DbcMessage::signal(const std::string& n) const {
  for (const DbcSignal& s : signals) {
    if (s.name == n) {
      return &s;
    }
  }
  return nullptr;
}

const DbcSignal* DbcMessage::multiplexer() const {
  for (const DbcSignal& s : signals) {
    if (s.multiplexer) {
      return &s;
    }
  }
  return nullptr;
}

const DbcMessage* Dbc::message(const std::string& n) const {
  for (const DbcMessage& m : messages) {
    if (m.name == n) {
      return &m;
    }
  }
  return nullptr;
}

const DbcMessage* Dbc::message(std::uint32_t id, bool extended) const {
  for (const DbcMessage& m : messages) {
    if (m.id == id && m.extended == extended) {
      return &m;
    }
  }
  return nullptr;
}

Dbc Dbc::from_file(const std::filesystem::path& file) {
  std::ifstream in(file, std::ios::binary);
  if (!in) {
    throw std::invalid_argument("cannot read DBC '" + path_to_utf8(file) + "'");
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return parse(ss.str(), path_to_utf8(file.filename()));
}

Dbc Dbc::parse(const std::string& text, const std::string& source) {
  Dbc dbc;
  Lexer lex(text, source);
  std::map<std::string, double> default_cycle;  // BA_DEF_DEF_ "GenMsgCycleTime"
  std::map<std::uint32_t, double> cycle;        // raw id (with extended flag) -> ms
  std::map<std::uint32_t, int> frame_format;
  std::map<std::uint32_t, std::string> message_comments;
  std::map<std::pair<std::uint32_t, std::string>, std::string> signal_comments;
  std::map<std::pair<std::uint32_t, std::string>, int> value_types;
  std::vector<std::uint32_t> raw_ids;  // per message, as in the file
  DbcMessage* current = nullptr;

  for (;;) {
    const Token t = lex.next();
    if (t.type == Token::Type::End) {
      break;
    }
    if (t.type != Token::Type::Word) {
      continue;
    }
    const std::string& k = t.text;
    if (k == "VERSION") {
      dbc.version = lex.next().text;
    } else if (k == "NS_" || k == "BS_") {
      // NS_ : followed by an indented keyword list; BS_: [baudrate].
      lex.rest_of_line();
      if (k == "NS_") {
        for (;;) {
          lex.skip_space();
          const Token p = lex.peek();
          // Indented continuation lines hold the keyword list.
          if (p.type == Token::Type::Word && p.text.size() > 2 && p.text.back() == '_' && p.text != "BS_" &&
              p.text != "BU_" && p.text != "BO_") {
            lex.rest_of_line();
          } else {
            break;
          }
        }
      }
      current = nullptr;
    } else if (k == "BU_") {
      expect(lex, ":");
      std::istringstream names(lex.rest_of_line());
      std::string n;
      while (names >> n) {
        dbc.nodes.push_back(n);
      }
      current = nullptr;
    } else if (k == "BO_") {
      DbcMessage m;
      const auto raw = static_cast<std::uint32_t>(std::stoul(lex.next().text));
      m.extended = (raw & 0x80000000U) != 0;
      m.id = raw & 0x1FFFFFFFU;
      m.name = lex.next().text;
      expect(lex, ":");
      m.length = static_cast<int>(number(lex.next(), lex));
      m.transmitter = lex.next().text;
      if (m.transmitter == "Vector__XXX") {
        m.transmitter.clear();
      }
      if (m.length < 0 || m.length > 64) {
        lex.fail("message length must be 0..64 bytes", t);
      }
      dbc.messages.push_back(m);
      raw_ids.push_back(raw);
      current = &dbc.messages.back();
    } else if (k == "SG_") {
      if (current == nullptr) {
        lex.fail("SG_ outside a message", t);
      }
      DbcSignal s;
      s.name = lex.next().text;
      Token n = lex.next();
      if (n.text != ":") {
        // Multiplexing: M, or mN (mux value N), or mNM (extended multiplexing, read as mN).
        if (n.text == "M") {
          s.multiplexer = true;
        } else if (n.text.size() > 1 && n.text[0] == 'm') {
          std::size_t end = 1;
          while (end < n.text.size() && std::isdigit(static_cast<unsigned char>(n.text[end]))) {
            ++end;
          }
          s.mux_value = std::stoll(n.text.substr(1, end - 1));
          s.multiplexer = n.text.back() == 'M';
        }
        expect(lex, ":");
      }
      s.start_bit = static_cast<int>(number(lex.next(), lex));
      expect(lex, "|");
      s.length = static_cast<int>(number(lex.next(), lex));
      expect(lex, "@");
      // "@1+" may come as one number token "1" then "+" / "-", or "1+" is not split: handle both.
      Token order = lex.next();
      std::string order_text = order.text;
      if (order_text.size() == 1) {
        order_text += lex.next().text;
      }
      s.little_endian = order_text[0] == '1';
      s.is_signed = order_text.back() == '-';
      expect(lex, "(");
      s.factor = number(lex.next(), lex);
      expect(lex, ",");
      s.offset = number(lex.next(), lex);
      expect(lex, ")");
      expect(lex, "[");
      s.minimum = number(lex.next(), lex);
      expect(lex, "|");
      s.maximum = number(lex.next(), lex);
      expect(lex, "]");
      const Token unit = lex.next();
      s.unit = unit.text;
      // Receivers: words separated by commas, up to the end of the line.
      std::istringstream receivers(lex.rest_of_line());
      std::string r;
      while (std::getline(receivers, r, ',')) {
        std::istringstream words(r);
        std::string w;
        while (words >> w) {
          if (w != "Vector__XXX") {
            s.receivers.push_back(w);
          }
        }
      }
      if (s.length < 1 || s.length > 64 || s.start_bit < 0 || s.start_bit >= 8 * 64) {
        lex.fail("signal " + s.name + ": bad start bit or length", t);
      }
      if (s.factor == 0.0) {
        lex.fail("signal " + s.name + ": factor 0", t);
      }
      current->signals.push_back(s);
    } else if (k == "CM_") {
      const Token what = lex.next();
      if (what.text == "BO_") {
        const auto raw = static_cast<std::uint32_t>(std::stoul(lex.next().text));
        message_comments[raw] = lex.next().text;
      } else if (what.text == "SG_") {
        const auto raw = static_cast<std::uint32_t>(std::stoul(lex.next().text));
        const std::string sig = lex.next().text;
        signal_comments[{raw, sig}] = lex.next().text;
      }
      lex.skip_statement();
      current = nullptr;
    } else if (k == "BA_DEF_DEF_") {
      const Token name = lex.next();
      const Token value = lex.next();
      if (name.text == "GenMsgCycleTime" && value.type == Token::Type::Number) {
        default_cycle["GenMsgCycleTime"] = std::stod(value.text);
      }
      lex.skip_statement();
      current = nullptr;
    } else if (k == "BA_") {
      const Token name = lex.next();
      const Token scope = lex.next();
      if (scope.text == "BO_" && (name.text == "GenMsgCycleTime" || name.text == "VFrameFormat")) {
        const auto raw = static_cast<std::uint32_t>(std::stoul(lex.next().text));
        const double value = number(lex.next(), lex);
        if (name.text == "GenMsgCycleTime") {
          cycle[raw] = value;
        } else {
          frame_format[raw] = static_cast<int>(value);
        }
      }
      lex.skip_statement();
      current = nullptr;
    } else if (k == "SIG_VALTYPE_") {
      const auto raw = static_cast<std::uint32_t>(std::stoul(lex.next().text));
      const std::string sig = lex.next().text;
      Token colon = lex.next();
      if (colon.text == ":") {
        colon = lex.next();
      }
      value_types[{raw, sig}] = static_cast<int>(number(colon, lex));
      lex.skip_statement();
      current = nullptr;
    } else if (k == "BA_DEF_" || k == "VAL_" || k == "VAL_TABLE_" || k == "BO_TX_BU_" || k == "EV_" ||
               k == "SIG_GROUP_" || k == "BA_DEF_REL_" || k == "BA_REL_" || k == "BU_SG_REL_" ||
               k == "BA_DEF_DEF_REL_" || k == "ENVVAR_DATA_" || k == "SGTYPE_" || k == "SIG_TYPE_REF_" ||
               k == "CAT_DEF_" || k == "CAT_" || k == "FILTER" || k == "SIGTYPE_VALTYPE_") {
      lex.skip_statement();
      current = nullptr;
    }
  }
  const double fallback_cycle = default_cycle.count("GenMsgCycleTime") ? default_cycle["GenMsgCycleTime"] : 0.0;
  for (std::size_t i = 0; i < dbc.messages.size(); ++i) {
    DbcMessage& m = dbc.messages[i];
    const std::uint32_t raw = raw_ids[i];
    m.cycle_time = (cycle.count(raw) ? cycle[raw] : fallback_cycle) / 1000.0;
    const int format = frame_format.count(raw) ? frame_format[raw] : -1;
    m.fd = format == 14 || format == 15 || m.length > 8;
    if (message_comments.count(raw)) {
      m.comment = message_comments[raw];
    }
    for (DbcSignal& s : m.signals) {
      if (signal_comments.count({raw, s.name})) {
        s.comment = signal_comments[{raw, s.name}];
      }
      const auto vt = value_types.find({raw, s.name});
      if (vt != value_types.end()) {
        s.kind = vt->second == 1 ? DbcSignal::Kind::Float32 : vt->second == 2 ? DbcSignal::Kind::Float64
                                                                              : DbcSignal::Kind::Integer;
      }
      for (const int bit : bit_positions(s)) {
        if (bit < 0 || bit >= 8 * m.length) {
          throw std::invalid_argument(source + ": signal " + m.name + "." + s.name + " does not fit in " +
                                      std::to_string(m.length) + " bytes");
        }
      }
    }
  }
  return dbc;
}

std::string Dbc::to_text() const {
  std::ostringstream o;
  o << "VERSION \"" << version << "\"\n\nNS_ :\n\tCM_\n\tBA_DEF_\n\tBA_\n\tVAL_\n\tBA_DEF_DEF_\n\tSIG_VALTYPE_\n\n"
    << "BS_:\n\nBU_:";
  for (const std::string& n : nodes) {
    o << " " << n;
  }
  o << "\n\n";
  const auto raw_id = [](const DbcMessage& m) { return m.id | (m.extended ? 0x80000000U : 0U); };
  for (const DbcMessage& m : messages) {
    o << "BO_ " << raw_id(m) << " " << m.name << ": " << m.length << " "
      << (m.transmitter.empty() ? "Vector__XXX" : m.transmitter) << "\n";
    for (const DbcSignal& s : m.signals) {
      o << " SG_ " << s.name << " ";
      if (s.multiplexer && !s.mux_value) {
        o << "M ";
      } else if (s.mux_value) {
        o << "m" << *s.mux_value << (s.multiplexer ? "M " : " ");
      }
      o << ": " << s.start_bit << "|" << s.length << "@" << (s.little_endian ? "1" : "0") << (s.is_signed ? "-" : "+")
        << " (" << format_number(s.factor) << "," << format_number(s.offset) << ") [" << format_number(s.minimum)
        << "|" << format_number(s.maximum) << "] \"" << s.unit << "\" ";
      if (s.receivers.empty()) {
        o << "Vector__XXX";
      }
      for (std::size_t i = 0; i < s.receivers.size(); ++i) {
        o << (i ? "," : "") << s.receivers[i];
      }
      o << "\n";
    }
    o << "\n";
  }
  for (const DbcMessage& m : messages) {
    if (!m.comment.empty()) {
      o << "CM_ BO_ " << raw_id(m) << " \"" << m.comment << "\";\n";
    }
    for (const DbcSignal& s : m.signals) {
      if (!s.comment.empty()) {
        o << "CM_ SG_ " << raw_id(m) << " " << s.name << " \"" << s.comment << "\";\n";
      }
    }
  }
  o << "BA_DEF_ BO_ \"GenMsgCycleTime\" INT 0 65535;\n";
  o << "BA_DEF_ BO_ \"VFrameFormat\" ENUM \"StandardCAN\",\"ExtendedCAN\",\"reserved\",\"reserved\","
       "\"reserved\",\"reserved\",\"reserved\",\"reserved\",\"reserved\",\"reserved\",\"reserved\",\"reserved\","
       "\"reserved\",\"reserved\",\"StandardCAN_FD\",\"ExtendedCAN_FD\";\n";
  o << "BA_DEF_DEF_ \"GenMsgCycleTime\" 0;\n";
  o << "BA_DEF_DEF_ \"VFrameFormat\" \"StandardCAN\";\n";
  for (const DbcMessage& m : messages) {
    if (m.cycle_time > 0.0) {
      o << "BA_ \"GenMsgCycleTime\" BO_ " << raw_id(m) << " " << std::lround(m.cycle_time * 1000.0) << ";\n";
    }
    if (m.fd) {
      o << "BA_ \"VFrameFormat\" BO_ " << raw_id(m) << " " << (m.extended ? 15 : 14) << ";\n";
    }
  }
  for (const DbcMessage& m : messages) {
    for (const DbcSignal& s : m.signals) {
      if (s.kind != DbcSignal::Kind::Integer) {
        o << "SIG_VALTYPE_ " << raw_id(m) << " " << s.name << " : " << (s.kind == DbcSignal::Kind::Float32 ? 1 : 2)
          << ";\n";
      }
    }
  }
  return o.str();
}

}  // namespace vsim
