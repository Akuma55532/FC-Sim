#pragma once
#include <memory>
#include <rviz_common/panel.hpp>
namespace flight_rviz_plugins {
class FlightPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit FlightPanel(QWidget * parent=nullptr);
  ~FlightPanel() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
