// ===== src/presentation/qt/presenters/AdminPresenter.hpp =====
#pragma once

#include <QObject>

class AdminViewModel;

/// Презентер для AdminView.
/// В MVP — тонкая обёртка; место для будущей логики (права доступа, аудит).
class AdminPresenter final : public QObject {
    Q_OBJECT
public:
    explicit AdminPresenter(AdminViewModel* vm,
                            QObject*        parent = nullptr);

public slots:
    void onLoad();

private:
    AdminViewModel* vm_;
};
