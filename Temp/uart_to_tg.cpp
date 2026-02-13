#include <bits/stdc++.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ---------------------- Config ----------------------
static const char* PORT = "/dev/ttyAMA0";
static const int   BAUD = 115200;

static const double SEND_EVERY_SEC = 20.0;
static const int MAX_LINES_PER_MSG = 2;
static const int MAX_CHARS = 3500;

static const bool DEBUG = true;

// ---------------------- Globals ----------------------
static std::atomic<bool> running{true};
static std::mutex uart_mtx;

static std::string BOT_TOKEN;
static std::string CHAT_ID;

// ---------------------- Time helpers ----------------------
static double now_sec() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ---------------------- UART helpers (termios) ----------------------
static speed_t to_speed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B115200;
    }
}

class SerialPort {
public:
    explicit SerialPort(const char* path, int baud) {
        fd_ = ::open(path, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) throw std::runtime_error("Cannot open serial port");

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) throw std::runtime_error("tcgetattr failed");

        cfsetospeed(&tty, to_speed(baud));
        cfsetispeed(&tty, to_speed(baud));

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;                            // no signaling chars, no echo
        tty.c_oflag = 0;

        tty.c_cc[VMIN]  = 0; // non-blocking read with timeout via select()
        tty.c_cc[VTIME] = 0;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);      // no SW flow control
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);           // no parity
        tty.c_cflag &= ~CSTOPB;                      // 1 stop bit
        tty.c_cflag &= ~CRTSCTS;                     // no HW flow control

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) throw std::runtime_error("tcsetattr failed");
        tcflush(fd_, TCIFLUSH);
    }

    ~SerialPort() {
        if (fd_ >= 0) ::close(fd_);
    }

    void write_all(const uint8_t* data, size_t n) {
        size_t off = 0;
        while (off < n) {
            ssize_t w = ::write(fd_, data + off, n - off);
            if (w < 0) throw std::runtime_error("UART write failed");
            off += (size_t)w;
        }
        ::tcdrain(fd_);
    }

    // Read line ending with '\n' with overall timeout_sec (like Python timeout=1 on readline).
    std::string readline(double timeout_sec) {
        std::string out;
        double deadline = now_sec() + timeout_sec;

        while (now_sec() < deadline) {
            double remain = std::max(0.0, deadline - now_sec());
            timeval tv{};
            tv.tv_sec = (int)remain;
            tv.tv_usec = (int)((remain - tv.tv_sec) * 1e6);

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(fd_, &rfds);

            int r = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
            if (r < 0) throw std::runtime_error("select() failed");
            if (r == 0) break; // timeout

            char buf[256];
            ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n < 0) throw std::runtime_error("UART read failed");
            if (n == 0) continue;

            if (DEBUG) {
                std::string raw(buf, buf + n);
                std::cerr << "UART raw chunk: ";
                for (unsigned char c : raw) std::cerr << std::hex << std::setw(2) << std::setfill('0') << (int)c << " ";
                std::cerr << std::dec << "\n";
            }

            for (ssize_t i = 0; i < n; i++) {
                out.push_back(buf[i]);
                if (buf[i] == '\n') return out;
            }
        }

        return out; // may be empty or partial
    }

private:
    int fd_{-1};
};

// ---------------------- Telegram (libcurl) ----------------------
static size_t curl_write_cb(void* contents, size_t size, size_t nmemb, void* userp) {
    size_t total = size * nmemb;
    std::string* s = (std::string*)userp;
    s->append((char*)contents, total);
    return total;
}

static std::string url_encode(CURL* curl, const std::string& s) {
    char* enc = curl_easy_escape(curl, s.c_str(), (int)s.size());
    if (!enc) return "";
    std::string out(enc);
    curl_free(enc);
    return out;
}

static json http_get_json(const std::string& url, const std::vector<std::pair<std::string,std::string>>& params, long timeout_sec) {
    CURL* curl = curl_easy_init();
    if (!curl) throw std::runtime_error("curl_easy_init failed");

    std::string full = url;
    if (!params.empty()) {
        full += "?";
        bool first = true;
        for (auto& kv : params) {
            if (!first) full += "&";
            first = false;
            full += kv.first + "=" + url_encode(curl, kv.second);
        }
    }

    std::string resp;
    curl_easy_setopt(curl, CURLOPT_URL, full.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_sec + 10);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);

    // Обычно достаточно системных CA. Если встраиваемое окружение без CA — надо отдельно настраивать.
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);

    CURLcode rc = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    curl_easy_cleanup(curl);

    if (rc != CURLE_OK) {
        throw std::runtime_error(std::string("HTTP GET failed: ") + curl_easy_strerror(rc));
    }
    if (http_code < 200 || http_code >= 300) {
        throw std::runtime_error("HTTP GET non-2xx: " + std::to_string(http_code) + " body: " + resp.substr(0, 300));
    }

    return json::parse(resp);
}

static json http_post_form_json(const std::string& url, const std::vector<std::pair<std::string,std::string>>& form, long timeout_sec) {
    CURL* curl = curl_easy_init();
    if (!curl) throw std::runtime_error("curl_easy_init failed");

    std::string postfields;
    bool first = true;
    for (auto& kv : form) {
        if (!first) postfields += "&";
        first = false;
        postfields += kv.first + "=" + url_encode(curl, kv.second);
    }

    std::string resp;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postfields.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_sec);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);

    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);

    CURLcode rc = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    curl_easy_cleanup(curl);

    if (DEBUG) {
        std::cerr << "TG send: " << http_code << " " << resp.substr(0, 300) << "\n";
    }

    if (rc != CURLE_OK) {
        throw std::runtime_error(std::string("HTTP POST failed: ") + curl_easy_strerror(rc));
    }
    if (http_code < 200 || http_code >= 300) {
        throw std::runtime_error("HTTP POST non-2xx: " + std::to_string(http_code) + " body: " + resp.substr(0, 300));
    }

    return json::parse(resp);
}

static json tg_get_updates(std::optional<long long> offset, int timeout_sec) {
    std::string url = "https://api.telegram.org/bot" + BOT_TOKEN + "/getUpdates";
    std::vector<std::pair<std::string,std::string>> params;
    params.push_back({"timeout", std::to_string(timeout_sec)});
    if (offset.has_value()) params.push_back({"offset", std::to_string(*offset)});
    return http_get_json(url, params, timeout_sec);
}

static void tg_send(const std::string& text) {
    std::string url = "https://api.telegram.org/bot" + BOT_TOKEN + "/sendMessage";
    std::vector<std::pair<std::string,std::string>> form = {
        {"chat_id", CHAT_ID},
        {"text", text},
        {"disable_web_page_preview", "true"}
    };
    (void)http_post_form_json(url, form, 10);
}

// ---------------------- Protocol helpers ----------------------
static std::string clean_ascii(const std::string& line) {
    std::string out;
    out.reserve(line.size());
    for (unsigned char b : line) {
        if (b == 9 || b == 10 || b == 13 || (b >= 32 && b <= 126)) out.push_back((char)b);
    }
    // trim
    auto is_space = [](unsigned char c){ return std::isspace(c); };
    size_t a = 0, b = out.size();
    while (a < b && is_space((unsigned char)out[a])) a++;
    while (b > a && is_space((unsigned char)out[b-1])) b--;
    return out.substr(a, b - a);
}

static std::vector<uint8_t> make_frame(const std::string& cmd, const std::optional<std::string>& data) {
    std::string frame_str;
    if (!data.has_value()) frame_str = " " + cmd + "  ";
    else frame_str = " " + cmd + " " + *data + " ";

    std::vector<uint8_t> out;
    out.reserve(1 + frame_str.size() + 1);
    out.push_back(0x02);
    out.insert(out.end(), frame_str.begin(), frame_str.end());
    out.push_back(0x0A);
    return out;
}

static void uart_send_cmd(SerialPort& ser, const std::string& cmd, const std::optional<std::string>& data = std::nullopt) {
    auto frame = make_frame(cmd, data);

    if (DEBUG) {
        std::cerr << "UART TX bytes: ";
        for (uint8_t c : frame) std::cerr << std::hex << std::setw(2) << std::setfill('0') << (int)c << " ";
        std::cerr << std::dec << "\n";
    }

    std::lock_guard<std::mutex> lk(uart_mtx);
    ser.write_all(frame.data(), frame.size());
}

// ---------------------- TG command loop thread ----------------------
static void tg_command_loop(SerialPort& ser) {
    std::optional<long long> offset;

    while (running.load()) {
        try {
            json js = tg_get_updates(offset, 30);
            if (!js.value("ok", false)) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            auto results = js.value("result", json::array());
            for (auto& upd : results) {
                long long upd_id = upd.value("update_id", 0LL);
                offset = upd_id + 1;

                json msg;
                if (upd.contains("message")) msg = upd["message"];
                else if (upd.contains("edited_message")) msg = upd["edited_message"];
                else continue;

                std::string chat_id = "";
                try {
                    chat_id = std::to_string(msg["chat"]["id"].get<long long>());
                } catch (...) {
                    continue;
                }

                std::string text = msg.value("text", "");
                // trim
                auto trim = [](std::string s){
                    auto is_space = [](unsigned char c){ return std::isspace(c); };
                    size_t a = 0, b = s.size();
                    while (a < b && is_space((unsigned char)s[a])) a++;
                    while (b > a && is_space((unsigned char)s[b-1])) b--;
                    return s.substr(a, b-a);
                };
                text = trim(text);

                if (chat_id != CHAT_ID) continue;

                if (text == "/start" || text == "start") {
                    uart_send_cmd(ser, "start");
                    tg_send("OK: start отправлен в UART");
                } else if (text == "/stop" || text == "stop") {
                    uart_send_cmd(ser, "stop");
                    tg_send("OK: stop отправлен в UART");
                } else if (text.rfind("/send ", 0) == 0) {
                    std::string payload = trim(text.substr(6));
                    if (payload.empty()) {
                        tg_send("Формат: /send <cmd> [data]");
                        continue;
                    }
                    std::string cmd;
                    std::optional<std::string> data;
                    auto pos = payload.find(' ');
                    if (pos == std::string::npos) cmd = payload;
                    else {
                        cmd = payload.substr(0, pos);
                        data = trim(payload.substr(pos + 1));
                        if (data->empty()) data.reset();
                    }
                    uart_send_cmd(ser, cmd, data);
                    tg_send("OK: отправлено: " + cmd + (data ? (" " + *data) : ""));
                } else if (text == "/help" || text == "help") {
                    tg_send(
                        "Команды:\n"
                        "/start — отправить start\n"
                        "/stop — отправить stop\n"
                        "/send <cmd> [data] — отправить произвольную команду"
                    );
                }
            }
        } catch (const std::exception& e) {
            if (DEBUG) std::cerr << "TG poll ERROR: " << e.what() << "\n";
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
}

// ---------------------- main ----------------------
int main() {
    const char* tok = std::getenv("BOT_TOKEN");
    const char* cid = std::getenv("CHAT_ID");
    if (!tok || !cid) {
        std::cerr << "Need env BOT_TOKEN and CHAT_ID\n";
        return 1;
    }
    BOT_TOKEN = tok;
    CHAT_ID = cid;

    curl_global_init(CURL_GLOBAL_DEFAULT);

    try {
        SerialPort ser(PORT, BAUD);

        if (DEBUG) {
            std::cerr << "UART opened: " << PORT << " @ " << BAUD << "\n";
            std::cerr << "CHAT_ID=" << CHAT_ID << "\n";
        }

        std::thread t([&]{ tg_command_loop(ser); });
        t.detach();

        // initial start
        {
            std::lock_guard<std::mutex> lk(uart_mtx);
            auto fr = make_frame("start", std::nullopt);
            ser.write_all(fr.data(), fr.size());
        }
        if (DEBUG) std::cerr << "UART TX: start\n";

        std::deque<std::string> buf;
        double last_send = now_sec();

        while (true) {
            std::string raw = ser.readline(1.0);

            if (DEBUG && !raw.empty()) {
                std::cerr << "UART raw: " << raw << "\n";
            }

            if (!raw.empty()) {
                std::string s = clean_ascii(raw);
                if (DEBUG) std::cerr << "UART text: " << std::quoted(s) << "\n";
                if (!s.empty()) buf.push_back(s);
            }

            double n = now_sec();
            if (!buf.empty() && (n - last_send >= SEND_EVERY_SEC || (int)buf.size() >= MAX_LINES_PER_MSG)) {
                std::vector<std::string> out_lines;
                out_lines.reserve(MAX_LINES_PER_MSG);

                int total = 0;
                while (!buf.empty() && (int)out_lines.size() < MAX_LINES_PER_MSG) {
                    std::string line = buf.front();
                    buf.pop_front();
                    if (total + (int)line.size() + 1 > MAX_CHARS) {
                        // вернуть обратно
                        buf.push_front(line);
                        break;
                    }
                    out_lines.push_back(line);
                    total += (int)line.size() + 1;
                }

                std::string text;
                for (size_t i = 0; i < out_lines.size(); i++) {
                    if (i) text += "\n";
                    text += out_lines[i];
                }

                if (DEBUG) {
                    std::cerr << "Sending " << out_lines.size() << " lines, " << text.size() << " chars\n";
                }

                try {
                    tg_send(text);
                    if (DEBUG) std::cerr << "TG send OK\n";
                } catch (const std::exception& e) {
                    if (DEBUG) std::cerr << "TG send ERROR: " << e.what() << "\n";
                    // вернуть строки обратно в начало буфера
                    for (auto it = out_lines.rbegin(); it != out_lines.rend(); ++it) {
                        buf.push_front(*it);
                    }
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }

                last_send = n;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "FATAL: " << e.what() << "\n";
        curl_global_cleanup();
        return 2;
    }

    curl_global_cleanup();
    return 0;
}
