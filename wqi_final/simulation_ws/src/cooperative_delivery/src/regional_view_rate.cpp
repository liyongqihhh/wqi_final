#include <cmath>

#include <QTimer>
#include <gazebo/common/Console.hh>
#include <gazebo/gui/GuiEvents.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/GuiPlugin.hh>

namespace cooperative_delivery
{
// Limit only the human viewport, never physics or simulated sensor updates.
class RegionalViewRate : public gazebo::GUIPlugin
{
public:
  RegionalViewRate()
  {
    setFixedSize(0, 0);
    setAttribute(Qt::WA_TransparentForMouseEvents);
    QObject::connect(
      &timer_, &QTimer::timeout, this, [this]() {
        if (gazebo::gui::get_active_camera()) {
          gazebo::gui::Events::setRenderRate(rate_);
          gzmsg << "Regional viewport limited to " << rate_ << " FPS\n";
          timer_.stop();
        }
      });
  }

  void Load(sdf::ElementPtr sdf) override
  {
    if (sdf && sdf->HasElement("fps")) {
      const auto rate = sdf->Get<double>("fps");
      if (std::isfinite(rate) && rate >= 5.0 && rate <= 30.0) {
        rate_ = rate;
      }
    }
    timer_.start(500);
  }

private:
  QTimer timer_;
  double rate_{10.0};
};

GZ_REGISTER_GUI_PLUGIN(RegionalViewRate)
}  // namespace cooperative_delivery
