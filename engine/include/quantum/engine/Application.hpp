#pragma once

namespace quantum::editor
{
    struct PreviewSmokeOptions;
}

namespace quantum::engine
{
    class Application
    {
    public:
        [[nodiscard]] int run();
        [[nodiscard]] int run(
            const editor::PreviewSmokeOptions& previewSmokeOptions);

    private:
        [[nodiscard]] int runImpl(
            const editor::PreviewSmokeOptions* previewSmokeOptions);
    };
}
