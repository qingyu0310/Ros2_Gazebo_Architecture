/**
 * @file keyboard.hpp
 * @author qingyu
 * @brief 终端键盘：非阻塞读方向键（Keyboard），加"按住走、松手停"的遥控映射（Teleop）
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 * @note 这一层没有 ROS、没有 Eigen：键盘就是一路设备输入，跟机器人无关，别的地方也能用
 * @note 为什么不用 getchar / std::cin：节点是定时器驱动的，读键盘不能把那一拍卡住。
 *       所以把终端设成原始模式的非阻塞读（VMIN=0 / VTIME=0），读不到立刻返回"没键"
 * @note 上下左右是转义序列，非阻塞读下三个字节可能落在不同的 poll() 里，所以状态机得跨调用
 *       记住"上一个字节是 ESC [ 还是 ESC O"，不能指望一次读齐。两种前缀都认：
 *       ESC [ A/B/C/D（普通模式，CSI）和 ESC O A/B/C/D（应用光标键模式，SS3 / DECCKM）
 * @note 关行缓冲和回显（ICANON / ECHO）在析构里恢复。进程要是被 SIGKILL 掉就恢复不了，
 *       那个终端得敲一下 reset
 * @note 两个类分工：Keyboard 只管"现在按了什么"，Teleop 管"按着这个键该给多少速度"。
 *       跟 modules::motor 那边一样，模块拿着状态，节点只负责喂时间和把结果发出去
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <termios.h>
#include <unistd.h>

namespace modules::keyboard
{

/**
 * @brief 认识的那几个键
 *
 * @note kNone 既是"这一拍没按"也是"按了个不认识的键" —— 对遥控来说两者一样：不用改速度
 */
enum class Key : std::uint8_t
{
    kNone  = 0,
    kUp    = 1,
    kDown  = 2,
    kLeft  = 3,
    kRight = 4
};

/**
 * @brief 终端键盘：原始模式 + 非阻塞读，把方向键还原成 Key
 *
 * @note 非拷贝：它手里攥着终端设置，拷一份出来析构时会把模式恢复两次
 */
class Keyboard
{
public:
    Keyboard() = default;

    /**
     * @brief 析构时恢复终端设置
     */
    ~Keyboard() { close(); }

    Keyboard(const Keyboard&)            = delete;
    Keyboard& operator=(const Keyboard&) = delete;

    /**
     * @brief 把 stdin 切成原始模式、非阻塞
     *
     * @return bool true 成功；false 说明 stdin 不是终端（被重定向到管道/文件了）或者没权限
     *
     * @note 只关 ICANON 和 ECHO，不动 ISIG：Ctrl-C 还得能杀进程
     */
    bool open()
    {
        if (opened_)
        {
            return true;
        }

        // 不是 tty 就直接失败，别装成"读到了键盘"。ros2 launch 里跑、或者被重定向时就是这种
        if (::tcgetattr(STDIN_FILENO, &saved_) != 0)
        {
            return false;
        }

        termios raw = saved_;
        raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);   // 不等回车、按键不回显
        raw.c_cc[VMIN]  = 0;                                    // 没数据就立刻返回，别阻塞
        raw.c_cc[VTIME] = 0;

        if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
        {
            return false;
        }

        opened_ = true;
        return true;
    }

    /**
     * @brief 恢复终端原来的设置
     *
     * @note 幂等，析构里也会调
     */
    void close()
    {
        if (!opened_)
        {
            return;
        }
        ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
        opened_ = false;
    }

    /**
     * @brief 终端是不是已经切到原始模式了
     */
    bool is_open() const { return opened_; }

    /**
     * @brief 把这一拍能读到的字节全读掉，返回最后一个认出来的键
     *
     * @return Key 这一拍最后认出来的键；没按键、或者按的是不认识的键，返回 Key::kNone
     *
     * @note 一次读一堆而不是一个字节就返回：终端按住不放时按键会连着重复，一次挤进来好几个，
     *       只处理第一个等于把后面的扔了。这里取"最后一个"——按键重复时它们本来也是同一个键
     * @note 转义序列读到一半（只收到 ESC）时返回 kNone：宁可这一拍不动，也别把半个序列当成键
     */
    Key poll()
    {
        if (!opened_)
        {
            return Key::kNone;
        }

        std::uint8_t byte       = 0;
        Key          last_key   = Key::kNone;

        for (std::size_t i = 0; i < kMaxBytesPerPoll; ++i)
        {
            const ssize_t got = ::read(STDIN_FILENO, &byte, 1);
            if (got <= 0)
            {
                break;   // 0：这一拍没键了；-1：出错或被信号打断，都当没键
            }

            const Key decoded = feed(byte);
            if (decoded != Key::kNone)
            {
                last_key = decoded;
            }
        }

        return last_key;
    }

private:
    // 一拍最多吃这么多字节。按键重复最猛也就几十 Hz，64 够用了；同时也是防呆，
    // 万一 stdin 被接到一个一直有数据的管道上，不至于在这一拍里转不出来
    static constexpr std::size_t kMaxBytesPerPoll = 64;

    // 转义序列的解析位置：三个字节可能分三次 poll() 到
    enum class Escape : std::uint8_t
    {
        kGround = 0,   // 不在序列里
        kAfterEsc,     // 刚收到 ESC
        kAfterCsi,     // 刚收到 ESC [，等最后一个字节
        kAfterSs3      // 刚收到 ESC O，等最后一个字节
    };

    bool    opened_{false};
    termios saved_{};
    Escape  escape_{Escape::kGround};

    /**
     * @brief 喂一个字节进状态机
     *
     * @param byte 收到的字节
     * @return Key 这个字节凑齐了一个方向键就返回它，否则 kNone
     */
    Key feed(std::uint8_t byte)
    {
        switch (escape_)
        {
            case Escape::kGround:
                if (byte == kEsc)
                {
                    escape_ = Escape::kAfterEsc;
                }
                return Key::kNone;

            case Escape::kAfterEsc:
                // 方向键有两条路：ESC [ A（普通模式，CSI）和 ESC O A（开了应用光标键之后，
                // SS3，DECCKM）。默认是前者，但终端设成后者的话只认 CSI 就会"方向键没反应"，
                // 所以两个都收。单独的 ESC（按 Esc 键）和别的转义序列都不管
                if (byte == '[')
                {
                    escape_ = Escape::kAfterCsi;
                }
                else if (byte == 'O')
                {
                    escape_ = Escape::kAfterSs3;
                }
                else
                {
                    escape_ = Escape::kGround;
                }
                return Key::kNone;

            case Escape::kAfterCsi:
            case Escape::kAfterSs3:
                escape_ = Escape::kGround;
                return decode_final(byte);
        }

        return Key::kNone;
    }

    /**
     * @brief 转义序列的最后一个字节 -> 方向键
     *
     * @param byte ESC [ 或 ESC O 后面那个字节
     * @return Key 认识的方向键；其它序列（PageUp、Home、F1~F4 之类）返回 kNone
     *
     * @note CSI 和 SS3 的最后一个字节是同一套字母，所以两条路共用这个解码
     */
    static Key decode_final(std::uint8_t byte)
    {
        switch (byte)
        {
            case 'A':
                return Key::kUp;
            case 'B':
                return Key::kDown;
            case 'C':
                return Key::kRight;
            case 'D':
                return Key::kLeft;
            default:
                return Key::kNone;
        }
    }

    static constexpr std::uint8_t kEsc = 0x1b;
};

/**
 * @brief 遥控的一拍：这一拍按了什么键、离上次按键过了多久
 */
struct KeyTick
{
    Key    key{Key::kNone};
    double key_age_s{std::numeric_limits<double>::infinity()};   // 从没按过就是无穷大
};

/**
 * @brief 遥控输出：给底盘的速度指令
 */
struct KeyOutput
{
    double linear_mps{0.0};      // 沿 +x
    double angular_radps{0.0};   // 绕 +z，逆时针为正
    bool   active{false};        // false = 松手超时了，输出的是 0
};

/**
 * @brief "按住走、松手停"的遥控映射：按键 -> 底盘速度
 *
 * @note 一次只认一个键：终端按住两个方向键时只会重复最后按下的那个，所以别指望斜着走
 * @note 松手判定靠超时：终端读不到"松开"这个事件，只能靠一段时间没收到按键来推断
 * @note 键位：上 = +linear，下 = -linear，左 = +angular（左转），右 = -angular
 */
class Teleop
{
public:
    /**
     * @brief 设按住一个方向键时给的线速度大小
     *
     * @param mps 线速度，m/s，正数
     */
    void set_linear_speed(double mps) { linear_speed_mps_ = mps; }

    /**
     * @brief 设按住左右键时给的角速度大小
     *
     * @param radps 角速度，rad/s，正数
     */
    void set_angular_speed(double radps) { angular_speed_radps_ = radps; }

    /**
     * @brief 设松手判定的超时
     *
     * @param timeout_s 秒
     *
     * @note 这个值必须大于终端的按键重复首次延迟，否则按住不放的时候首个延迟里收不到按键，
     *       车会一顿一顿。代价是松手后最多再走这么久才停（终端读不到"松开"事件）。
     *       本机实测延迟 500 ms（xset q: auto repeat delay），所以给 0.7 s 留 200 ms 余量；
     *       想让它更跟手，先跑 xset r rate 250 40 把延迟压到 250 ms，再把这里调到 0.4 左右
     */
    void set_key_timeout(double timeout_s) { key_timeout_s_ = timeout_s; }

    /**
     * @brief 跑一拍：按键更新目标速度，再看这个目标还新不新鲜
     *
     * @param tick 这一拍的按键和按键年龄（年龄由节点用 steady_clock 算，模块不碰时钟）
     */
    void update(const KeyTick& tick)
    {
        if (tick.key != Key::kNone)
        {
            latch(tick.key);
        }

        const bool fresh = tick.key_age_s <= key_timeout_s_;

        // 超时就把目标清掉：再按同一个键的时候要重新给速度，不能拿松手前的旧目标接着跑
        if (!fresh)
        {
            linear_mps_  = 0.0;
            angular_radps_ = 0.0;
        }

        output_               = KeyOutput{};
        output_.active        = fresh;
        output_.linear_mps    = fresh ? linear_mps_ : 0.0;
        output_.angular_radps = fresh ? angular_radps_ : 0.0;
    }

    /**
     * @brief 最近一拍算出来的输出
     */
    const KeyOutput& output() const { return output_; }

private:
    double linear_speed_mps_{1.0};        // 按住上下时的线速度大小（轮子上是 25 rad/s，不到转速上限 60 的一半）
    double angular_speed_radps_{2.0};     // 按住左右时的角速度大小
    double key_timeout_s_{0.70};          // 松手判定，见 set_key_timeout

    double linear_mps_{0.0};              // 当前锁定的目标速度
    double angular_radps_{0.0};

    KeyOutput output_{};

    /**
     * @brief 一个键按下时锁定的目标速度
     *
     * @param key 按下的键
     *
     * @note 直线键会把角速度清零、转向键会把线速度清零：一次只认一个键，不清零就成了"按着上
     *       再点一下左，之后就永远斜着走"
     */
    void latch(Key key)
    {
        linear_mps_    = 0.0;
        angular_radps_ = 0.0;

        switch (key)
        {
            case Key::kUp:
                linear_mps_ = linear_speed_mps_;
                break;
            case Key::kDown:
                linear_mps_ = -linear_speed_mps_;
                break;
            case Key::kLeft:
                angular_radps_ = angular_speed_radps_;
                break;
            case Key::kRight:
                angular_radps_ = -angular_speed_radps_;
                break;
            case Key::kNone:
                break;
        }
    }
};

}   // namespace modules::keyboard
