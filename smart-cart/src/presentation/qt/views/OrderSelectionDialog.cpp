#include "views/OrderSelectionDialog.hpp"

#include <nlohmann/json.hpp>

#include <QDateTime>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QVBoxLayout>

#include <fstream>

using json = nlohmann::json;

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Читает поля order_id и title из JSON-файла (первые 4 КБ).
QString OrderSelectionDialog::orderSummary(const QString& filePath)
{
    if (!filePath.endsWith(".json", Qt::CaseInsensitive)) {
        return {};
    }
    try {
        std::ifstream in(filePath.toStdString());
        if (!in.is_open()) return {};

        json j;
        in >> j;
        if (!j.is_object()) return {};

        QString id, title;
        if (j.contains("order_id") && j["order_id"].is_string()) {
            id = QString::fromStdString(j["order_id"].get<std::string>());
        }
        if (j.contains("title") && j["title"].is_string()) {
            title = QString::fromStdString(j["title"].get<std::string>());
        }
        if (id.isEmpty() && title.isEmpty()) return {};
        if (title.isEmpty()) return id;
        if (id.isEmpty()) return title;
        return id + QString::fromUtf8(" — ") + title;
    } catch (...) {
        return {};
    }
}

// ── Construction ──────────────────────────────────────────────────────────────

OrderSelectionDialog::OrderSelectionDialog(const QString& ordersDir,
                                           QWidget*       parent)
    : QDialog(parent)
    , ordersDir_(ordersDir)
{
    buildUi();
    populateList();
}

void OrderSelectionDialog::buildUi()
{
    setWindowTitle(QString::fromUtf8("Выбор заказа"));
    setMinimumSize(520, 420);
    setModal(true);

    // Тёмный стиль соответствует общему UI
    setStyleSheet(
        "QDialog { background: #1e1e2e; color: #cdd6f4; }"
        "QLabel  { color: #cdd6f4; }"
        "QLineEdit {"
        "  background: #313244; border: 1px solid #45475a;"
        "  border-radius: 6px; padding: 6px 10px; color: #cdd6f4; font-size: 14px; }"
        "QListWidget {"
        "  background: #181825; border: 1px solid #45475a;"
        "  border-radius: 6px; color: #cdd6f4; font-size: 13px; outline: none; }"
        "QListWidget::item { padding: 8px 12px; border-bottom: 1px solid #313244; }"
        "QListWidget::item:selected { background: #45475a; color: #89b4fa; }"
        "QListWidget::item:hover:!selected { background: #313244; }"
        "QPushButton {"
        "  background: #313244; border: none; border-radius: 6px;"
        "  padding: 8px 20px; color: #cdd6f4; font-size: 14px; }"
        "QPushButton:hover   { background: #45475a; }"
        "QPushButton:pressed { background: #585b70; }"
        "QPushButton#selectBtn { background: #89b4fa; color: #1e1e2e; font-weight: bold; }"
        "QPushButton#selectBtn:hover { background: #b4befe; }"
        "QPushButton#selectBtn:disabled { background: #45475a; color: #6c7086; }"
    );

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(16, 16, 16, 16);
    layout->setSpacing(10);

    // Заголовок
    auto* titleLabel = new QLabel(
        QString::fromUtf8("Директория: <b>%1</b>").arg(ordersDir_), this);
    titleLabel->setWordWrap(true);
    layout->addWidget(titleLabel);

    // Строка поиска
    auto* searchEdit = new QLineEdit(this);
    searchEdit->setPlaceholderText(QString::fromUtf8("🔍 Поиск по имени файла или заказу…"));
    connect(searchEdit, &QLineEdit::textChanged,
            this, &OrderSelectionDialog::onSearchTextChanged);
    layout->addWidget(searchEdit);

    // Список файлов
    listWidget_ = new QListWidget(this);
    listWidget_->setAlternatingRowColors(false);
    connect(listWidget_, &QListWidget::itemDoubleClicked,
            this, &OrderSelectionDialog::onItemDoubleClicked);
    connect(listWidget_, &QListWidget::itemSelectionChanged, this, [this]() {
        selectBtn_->setEnabled(listWidget_->currentItem() != nullptr);
    });
    layout->addWidget(listWidget_, 1);

    // Статус (кол-во файлов)
    statusLabel_ = new QLabel(this);
    statusLabel_->setStyleSheet("color: #6c7086; font-size: 12px;");
    layout->addWidget(statusLabel_);

    // Кнопки
    auto* btnRow = new QHBoxLayout;
    btnRow->setSpacing(8);

    auto* refreshBtn = new QPushButton(QString::fromUtf8("⟳ Обновить"), this);
    connect(refreshBtn, &QPushButton::clicked,
            this, &OrderSelectionDialog::onRefreshClicked);
    btnRow->addWidget(refreshBtn);

    auto* browseBtn = new QPushButton(QString::fromUtf8("📂 Обзор…"), this);
    connect(browseBtn, &QPushButton::clicked,
            this, &OrderSelectionDialog::onBrowseClicked);
    btnRow->addWidget(browseBtn);

    btnRow->addStretch();

    auto* cancelBtn = new QPushButton(QString::fromUtf8("Отмена"), this);
    connect(cancelBtn, &QPushButton::clicked, this, &QDialog::reject);
    btnRow->addWidget(cancelBtn);

    selectBtn_ = new QPushButton(QString::fromUtf8("Выбрать"), this);
    selectBtn_->setObjectName("selectBtn");
    selectBtn_->setEnabled(false);
    selectBtn_->setDefault(true);
    connect(selectBtn_, &QPushButton::clicked,
            this, &OrderSelectionDialog::onSelectClicked);
    btnRow->addWidget(selectBtn_);

    layout->addLayout(btnRow);
}

// ── File list population ──────────────────────────────────────────────────────

void OrderSelectionDialog::populateList(const QString& filter)
{
    listWidget_->clear();

    QDir dir(ordersDir_);
    if (!dir.exists()) {
        statusLabel_->setText(
            QString::fromUtf8("Директория не найдена: %1").arg(ordersDir_));
        return;
    }

    const QStringList nameFilters{
        "*.json", "*.JSON",
        "*.xlsx", "*.XLSX",
        "*.xls",  "*.XLS"
    };
    const auto entries = dir.entryInfoList(
        nameFilters,
        QDir::Files | QDir::Readable,
        QDir::Time  // сортировка по дате изменения, новые первые
    );

    int shown = 0;
    for (const QFileInfo& fi : entries) {
        // Фильтр поиска
        const QString lower = filter.toLower();
        if (!lower.isEmpty() &&
            !fi.fileName().toLower().contains(lower))
        {
            // Для JSON — проверяем ещё содержимое
            const QString summary = orderSummary(fi.filePath());
            if (!summary.toLower().contains(lower)) {
                continue;
            }
        }

        // Формируем строку отображения
        const QString date = fi.lastModified().toString("dd.MM.yyyy HH:mm");
        const qint64  kb   = (fi.size() + 1023) / 1024;

        QString display = fi.fileName();
        const QString summary = orderSummary(fi.filePath());
        if (!summary.isEmpty()) {
            display += QString::fromUtf8("   —   ") + summary;
        }
        display += QString::fromUtf8("\n%1   %2 КБ").arg(date).arg(kb);

        auto* item = new QListWidgetItem(display, listWidget_);
        item->setData(Qt::UserRole, fi.filePath()); // полный путь
        item->setToolTip(fi.filePath());
        ++shown;
    }

    statusLabel_->setText(
        QString::fromUtf8("Файлов: %1").arg(shown) +
        (shown != entries.size()
            ? QString::fromUtf8(" (из %1)").arg(entries.size())
            : QString{}));
}

// ── Slots ─────────────────────────────────────────────────────────────────────

void OrderSelectionDialog::onItemDoubleClicked(QListWidgetItem* item)
{
    if (!item) return;
    selectedPath_ = item->data(Qt::UserRole).toString();
    accept();
}

void OrderSelectionDialog::onSelectClicked()
{
    auto* item = listWidget_->currentItem();
    if (!item) return;
    selectedPath_ = item->data(Qt::UserRole).toString();
    accept();
}

void OrderSelectionDialog::onBrowseClicked()
{
    const QString path = QFileDialog::getOpenFileName(
        this,
        QString::fromUtf8("Выберите файл заказа"),
        ordersDir_,
        QString::fromUtf8("Заказ или BOM (*.json *.xlsx *.xls);;JSON (*.json);;Excel BOM (*.xlsx *.xls);;Все файлы (*)")
    );
    if (!path.isEmpty()) {
        selectedPath_ = path;
        accept();
    }
}

void OrderSelectionDialog::onRefreshClicked()
{
    populateList();
}

void OrderSelectionDialog::onSearchTextChanged(const QString& text)
{
    populateList(text);
}
