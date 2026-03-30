// ===== src/presentation/qt/presenters/WorkerPresenter.hpp =====
#pragma once

#include <QObject>

class WorkerViewModel;

namespace smartcart::infrastructure::hw::scanner {
    class MockScannerProvider;
}

/// Связывает физический сканер с WorkerViewModel.
/// В demoMode — принимает инжектированные штрихкоды.
class WorkerPresenter final : public QObject {
    Q_OBJECT
public:
    explicit WorkerPresenter(WorkerViewModel* vm,
                             QObject*         parent = nullptr);

    /// Эмулировать сканирование (для demo / тестов).
    void injectBarcode(const QString& barcode);

private:
    WorkerViewModel* vm_;
};
