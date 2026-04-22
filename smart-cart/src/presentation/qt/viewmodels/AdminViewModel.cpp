#include "AdminViewModel.hpp"
#include "../../../domain/entities/ModuleInfo.hpp"

#include <QStringList>

#include <algorithm>
#include <exception>
#include <vector>

using namespace smartcart::domain;
using namespace smartcart::application::ports;

namespace services = smartcart::application::services;

// ── Конвертация domain → Qt ──────────────────────────────────────────────────
static ModuleItem toItem(const ModuleInfo& m) {
    return {
        m.id,
        QString::fromStdString(m.serial),
        m.slotCount,
        QString::fromStdString(m.firmware),
        QString::fromStdString(std::string{toString(m.status)}),
        QString::fromStdString(std::string{toString(m.kind)})
    };
}

// ── Конвертация Qt → domain ──────────────────────────────────────────────────
static ModuleInfo toDomain(int id, const QString& serial, int slotCount,
                            const QString& firmware, const QString& status) {
    ModuleInfo m;
    m.id        = id;
    m.serial    = serial.trimmed().toStdString();
    m.slotCount = slotCount;
    m.firmware  = firmware.trimmed().toStdString();
    m.status    = moduleStatusFromString(status.toStdString());
    return m;
}

static QString joinInts(const std::vector<int>& values) {
    if (values.empty()) {
        return QString::fromUtf8("нет");
    }

    QStringList parts;
    for (const int value : values) {
        parts << QString::number(value);
    }
    return parts.join(QStringLiteral(", "));
}

static QString formatChannelMap(const std::vector<int>& trackedChannels,
                                const std::vector<int>& ignoredChannels,
                                const std::vector<int>& channelToSlotMap) {
    QStringList ignored;
    for (const int channel : ignoredChannels) {
        ignored << QString::number(channel);
    }

    QStringList lines;
    for (const int channel : trackedChannels) {
        if (ignoredChannels.end() !=
            std::find(ignoredChannels.begin(), ignoredChannels.end(), channel))
        {
            lines << QString::fromUtf8("channel %1 → игнорируется").arg(channel);
            continue;
        }

        int slotIndex = channel + 1;
        if (channel >= 0 && channel < static_cast<int>(channelToSlotMap.size())) {
            slotIndex = channelToSlotMap[static_cast<size_t>(channel)];
        }

        if (slotIndex <= 0) {
            lines << QString::fromUtf8("channel %1 → не назначен").arg(channel);
        } else {
            lines << QString::fromUtf8("channel %1 → slot %2")
                         .arg(channel)
                         .arg(slotIndex);
        }
    }

    if (!ignored.isEmpty()) {
        lines << QString::fromUtf8("ignored: %1").arg(ignored.join(QStringLiteral(", ")));
    }
    return lines.join('\n');
}

static QString formatDiagnostics(const services::AdminDiagnosticsSnapshot& snapshot) {
    QStringList lines;
    lines << QString::fromUtf8("STM32");
    lines << QString::fromUtf8("  UART: %1")
                 .arg(snapshot.stm32.uartOpen
                          ? QString::fromUtf8("открыт")
                          : QString::fromUtf8("закрыт"));
    lines << QString::fromUtf8("  Обмен: %1")
                 .arg(snapshot.stm32.pollingRunning
                          ? QString::fromUtf8("активен")
                          : QString::fromUtf8("ожидает старта"));
    lines << QString::fromUtf8("  Последнее EVT: %1 (%2)")
                 .arg(QString::fromStdString(snapshot.stm32.lastEvent),
                      snapshot.stm32.lastEventAt.empty()
                          ? QString::fromUtf8("нет времени")
                          : QString::fromStdString(snapshot.stm32.lastEventAt));
    lines << QString::fromUtf8("  Последний snapshot: %1 (%2)")
                 .arg(QString::fromStdString(snapshot.stm32.lastSnapshot),
                      snapshot.stm32.lastSnapshotAt.empty()
                          ? QString::fromUtf8("нет времени")
                          : QString::fromStdString(snapshot.stm32.lastSnapshotAt));
    lines << QString::fromUtf8("  Активные каналы: %1")
                 .arg(joinInts(snapshot.stm32.activeChannels));

    lines << QString();
    lines << QString::fromUtf8("Маппинг каналов");
    lines << formatChannelMap(snapshot.trackedChannels,
                              snapshot.ignoredChannels,
                              snapshot.channelToSlotMap);

    lines << QString();
    lines << QString::fromUtf8("Последние события");
    for (const auto& event : snapshot.recentEvents) {
        lines << QString::fromUtf8("#%1 %2 [%3] %4 — %5")
                     .arg(event.id)
                     .arg(QString::fromStdString(event.ts),
                          QString::fromStdString(event.level),
                          QString::fromStdString(event.code),
                          QString::fromStdString(event.message));
    }

    return lines.join('\n');
}

// ── Ctor ─────────────────────────────────────────────────────────────────────
AdminViewModel::AdminViewModel(IModuleRepository& repo,
                               services::AdminDiagnosticsService& diagnosticsSvc,
                               QObject* parent)
    : QObject(parent)
    , repo_(repo)
    , diagnosticsSvc_(diagnosticsSvc)
{}

// ── Private ──────────────────────────────────────────────────────────────────
void AdminViewModel::reload() {
    items_.clear();
    for (const auto& m : repo_.getAll())
        items_.push_back(toItem(m));
    emit modulesReset();
}

int AdminViewModel::findIndexById(int id) const {
    for (int i = 0; i < items_.size(); ++i)
        if (items_[i].id == id) return i;
    return -1;
}

// ── Public slots ─────────────────────────────────────────────────────────────
void AdminViewModel::load() {
    reload();
}

void AdminViewModel::addModule(const QString& serial, int slotCount,
                                const QString& firmware, const QString& status) {
    if (serial.trimmed().isEmpty()) {
        emit errorOccurred("Serial не может быть пустым");
        return;
    }
    if (repo_.existsBySerial(serial.trimmed().toStdString())) {
        emit errorOccurred("Serial уже существует");
        return;
    }
    repo_.add(toDomain(0, serial, slotCount, firmware, status));
    reload();
    emit infoOccurred("Модуль добавлен");
}

void AdminViewModel::updateModule(int id, const QString& serial, int slotCount,
                                   const QString& firmware, const QString& status) {
    if (findIndexById(id) < 0) {
        emit errorOccurred("Модуль не найден");
        return;
    }
    if (serial.trimmed().isEmpty()) {
        emit errorOccurred("Serial не может быть пустым");
        return;
    }
    if (repo_.existsBySerial(serial.trimmed().toStdString(), id)) {
        emit errorOccurred("Serial уже существует");
        return;
    }
    auto module = toDomain(id, serial, slotCount, firmware, status);
    if (const auto existing = repo_.getById(id); existing.has_value()) {
        module.kind = existing->kind;
    }
    repo_.update(module);
    reload();
    emit infoOccurred("Модуль обновлён");
}

void AdminViewModel::removeModule(int id) {
    if (findIndexById(id) < 0) {
        emit errorOccurred("Модуль не найден");
        return;
    }
    repo_.remove(id);
    reload();
    emit infoOccurred("Модуль удалён");
}

void AdminViewModel::refreshDiagnostics() {
    try {
        emit diagnosticsUpdated(formatDiagnostics(diagnosticsSvc_.snapshot(30)));
    } catch (const std::exception& ex) {
        emit errorOccurred(QString::fromUtf8("Не удалось обновить диагностику: ") +
                           QString::fromUtf8(ex.what()));
    } catch (...) {
        emit errorOccurred(QString::fromUtf8("Не удалось обновить диагностику"));
    }
}

void AdminViewModel::resetTestCart() {
    if (!diagnosticsSvc_.resetTestCart()) {
        emit errorOccurred(QString::fromUtf8("Не удалось сбросить тестовую тележку"));
        return;
    }

    emit infoOccurred(QString::fromUtf8("Тестовая тележка сброшена"));
    refreshDiagnostics();
}
