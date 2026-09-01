#include <quantum/editor/EditorIcons.hpp>
#include <quantum/editor/EditorStyle.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    struct ParsedStroke
    {
        std::vector<ImVec2> points;
        bool closed = false;
    };

    struct ParsedDrawing
    {
        std::vector<ParsedStroke> strokes;
        ImVec2 viewBoxMinimum{};
        ImVec2 viewBoxSize{};
        ImVec2 contentMinimum{};
        ImVec2 contentMaximum{};
        float strokeWidth = 0.0F;
    };

    constexpr std::array<std::string_view,
        static_cast<std::size_t>(quantum::editor::EditorIcon::Count)>
        editorIconFileNames{
            "mouse-pointer-2.svg",
            "move-3d.svg",
            "rotate-3d.svg",
            "orbit.svg",
            "hand.svg",
            "focus.svg",
            "camera.svg",
            "axis-3d.svg",
            "maximize.svg",
            "folder-open.svg",
            "save.svg",
            "undo-2.svg",
            "redo-2.svg",
            "eye.svg",
            "eye-off.svg",
            "lock.svg",
            "lock-open.svg",
            "zoom-in.svg",
            "zoom-out.svg",
            "play.svg",
            "pause.svg",
            "square-stop.svg",
            "settings.svg"
        };

    class NumberReader
    {
    public:
        explicit NumberReader(std::string_view text)
            : text_(text)
        {
        }

        void skipSeparators() noexcept
        {
            while (position_ < text_.size())
            {
                const unsigned char character = static_cast<unsigned char>(
                    text_[position_]);
                if (!std::isspace(character) && text_[position_] != ',')
                {
                    break;
                }
                ++position_;
            }
        }

        [[nodiscard]] bool atEnd() noexcept
        {
            skipSeparators();
            return position_ == text_.size();
        }

        [[nodiscard]] bool nextIsCommand() noexcept
        {
            skipSeparators();
            return position_ < text_.size()
                && std::isalpha(static_cast<unsigned char>(text_[position_])) != 0;
        }

        [[nodiscard]] char readCommand()
        {
            if (!nextIsCommand())
            {
                throw std::runtime_error("expected an SVG path command");
            }
            return text_[position_++];
        }

        [[nodiscard]] float readNumber()
        {
            skipSeparators();
            if (position_ == text_.size())
            {
                throw std::runtime_error("unexpected end of SVG path data");
            }

            const std::string remaining{text_.substr(position_)};
            char* end = nullptr;
            const float value = std::strtof(remaining.c_str(), &end);
            if (end == remaining.c_str() || !std::isfinite(value))
            {
                throw std::runtime_error("invalid number in SVG path data");
            }
            position_ += static_cast<std::size_t>(end - remaining.c_str());
            return value;
        }

    private:
        std::string_view text_;
        std::size_t position_ = 0;
    };

    [[nodiscard]] ImVec2 add(const ImVec2 left, const ImVec2 right) noexcept
    {
        return {left.x + right.x, left.y + right.y};
    }

    [[nodiscard]] ImVec2 readPoint(NumberReader& reader)
    {
        return {reader.readNumber(), reader.readNumber()};
    }

    void appendCubicBezier(
        std::vector<ImVec2>& points,
        const ImVec2 from,
        const ImVec2 controlOne,
        const ImVec2 controlTwo,
        const ImVec2 to)
    {
        constexpr int segmentCount = 12;
        for (int segment = 1; segment <= segmentCount; ++segment)
        {
            const float t = static_cast<float>(segment)
                / static_cast<float>(segmentCount);
            const float inverseT = 1.0F - t;
            const float fromWeight = inverseT * inverseT * inverseT;
            const float controlOneWeight =
                3.0F * inverseT * inverseT * t;
            const float controlTwoWeight =
                3.0F * inverseT * t * t;
            const float toWeight = t * t * t;
            points.push_back({
                fromWeight * from.x
                    + controlOneWeight * controlOne.x
                    + controlTwoWeight * controlTwo.x
                    + toWeight * to.x,
                fromWeight * from.y
                    + controlOneWeight * controlOne.y
                    + controlTwoWeight * controlTwo.y
                    + toWeight * to.y
            });
        }

        // Keep the final point bit-identical to the SVG command endpoint.
        points.back() = to;
    }

    void appendArc(
        std::vector<ImVec2>& points,
        const ImVec2 from,
        const ImVec2 to,
        float radiusX,
        float radiusY,
        const float rotationDegrees,
        const bool largeArc,
        const bool sweep)
    {
        radiusX = std::abs(radiusX);
        radiusY = std::abs(radiusY);
        if (radiusX == 0.0F || radiusY == 0.0F)
        {
            points.push_back(to);
            return;
        }
        if (from.x == to.x && from.y == to.y)
        {
            return;
        }

        constexpr double radiansPerDegree =
            std::numbers::pi_v<double> / 180.0;
        const double rotation =
            static_cast<double>(rotationDegrees) * radiansPerDegree;
        const double cosine = std::cos(rotation);
        const double sine = std::sin(rotation);
        const double halfX =
            (static_cast<double>(from.x) - static_cast<double>(to.x)) * 0.5;
        const double halfY =
            (static_cast<double>(from.y) - static_cast<double>(to.y)) * 0.5;
        const double transformedX = cosine * halfX + sine * halfY;
        const double transformedY = -sine * halfX + cosine * halfY;

        double rx = static_cast<double>(radiusX);
        double ry = static_cast<double>(radiusY);
        const double radiusScale =
            transformedX * transformedX / (rx * rx)
            + transformedY * transformedY / (ry * ry);
        if (radiusScale > 1.0)
        {
            const double scale = std::sqrt(radiusScale);
            rx *= scale;
            ry *= scale;
        }

        const double rxSquared = rx * rx;
        const double rySquared = ry * ry;
        const double transformedXSquared = transformedX * transformedX;
        const double transformedYSquared = transformedY * transformedY;
        const double denominator =
            rxSquared * transformedYSquared
            + rySquared * transformedXSquared;
        const double numerator = std::max(
            0.0,
            rxSquared * rySquared
                - rxSquared * transformedYSquared
                - rySquared * transformedXSquared
        );
        const double sign = largeArc == sweep ? -1.0 : 1.0;
        const double coefficient = denominator == 0.0
            ? 0.0
            : sign * std::sqrt(numerator / denominator);
        const double centerTransformedX =
            coefficient * rx * transformedY / ry;
        const double centerTransformedY =
            coefficient * -ry * transformedX / rx;
        const double centerX =
            cosine * centerTransformedX - sine * centerTransformedY
            + (static_cast<double>(from.x) + static_cast<double>(to.x)) * 0.5;
        const double centerY =
            sine * centerTransformedX + cosine * centerTransformedY
            + (static_cast<double>(from.y) + static_cast<double>(to.y)) * 0.5;

        const double unitStartX =
            (transformedX - centerTransformedX) / rx;
        const double unitStartY =
            (transformedY - centerTransformedY) / ry;
        const double unitEndX =
            (-transformedX - centerTransformedX) / rx;
        const double unitEndY =
            (-transformedY - centerTransformedY) / ry;
        const double startAngle = std::atan2(unitStartY, unitStartX);
        double angleDelta = std::atan2(
            unitStartX * unitEndY - unitStartY * unitEndX,
            unitStartX * unitEndX + unitStartY * unitEndY
        );
        if (!sweep && angleDelta > 0.0)
        {
            angleDelta -= 2.0 * std::numbers::pi_v<double>;
        }
        else if (sweep && angleDelta < 0.0)
        {
            angleDelta += 2.0 * std::numbers::pi_v<double>;
        }

        const int segmentCount = std::max(
            1,
            static_cast<int>(std::ceil(
                std::abs(angleDelta) / (std::numbers::pi_v<double> / 16.0)
            ))
        );
        for (int segment = 1; segment <= segmentCount; ++segment)
        {
            const double angle = startAngle
                + angleDelta * static_cast<double>(segment)
                    / static_cast<double>(segmentCount);
            const double ellipseX = rx * std::cos(angle);
            const double ellipseY = ry * std::sin(angle);
            points.push_back({
                static_cast<float>(
                    centerX + cosine * ellipseX - sine * ellipseY),
                static_cast<float>(
                    centerY + sine * ellipseX + cosine * ellipseY)
            });
        }

        // Keep the final point bit-identical to the SVG command endpoint.
        points.back() = to;
    }

    [[nodiscard]] std::vector<ParsedStroke> parsePath(
        const std::string_view pathData)
    {
        NumberReader reader(pathData);
        std::vector<ParsedStroke> strokes;
        ParsedStroke stroke;
        ImVec2 current{};
        ImVec2 subpathStart{};
        char command = 0;

        const auto finishStroke = [&]()
        {
            if (stroke.points.size() > 1)
            {
                strokes.push_back(std::move(stroke));
            }
            stroke = {};
        };

        while (!reader.atEnd())
        {
            if (reader.nextIsCommand())
            {
                command = reader.readCommand();
                if (command == 'z' || command == 'Z')
                {
                    stroke.closed = true;
                    finishStroke();
                    current = subpathStart;
                    command = 0;
                    continue;
                }
            }
            if (command == 0)
            {
                throw std::runtime_error("SVG path data has no active command");
            }

            const bool relative =
                command >= 'a' && command <= 'z';
            switch (static_cast<char>(std::toupper(
                static_cast<unsigned char>(command))))
            {
            case 'M':
            {
                ImVec2 point = readPoint(reader);
                if (relative)
                {
                    point = add(current, point);
                }
                finishStroke();
                current = point;
                subpathStart = point;
                stroke.points.push_back(point);
                command = relative ? 'l' : 'L';
                break;
            }
            case 'L':
            {
                ImVec2 point = readPoint(reader);
                if (relative)
                {
                    point = add(current, point);
                }
                stroke.points.push_back(point);
                current = point;
                break;
            }
            case 'H':
            {
                const float value = reader.readNumber();
                current.x = relative ? current.x + value : value;
                stroke.points.push_back(current);
                break;
            }
            case 'V':
            {
                const float value = reader.readNumber();
                current.y = relative ? current.y + value : value;
                stroke.points.push_back(current);
                break;
            }
            case 'C':
            {
                ImVec2 controlOne = readPoint(reader);
                ImVec2 controlTwo = readPoint(reader);
                ImVec2 point = readPoint(reader);
                if (relative)
                {
                    controlOne = add(current, controlOne);
                    controlTwo = add(current, controlTwo);
                    point = add(current, point);
                }
                appendCubicBezier(
                    stroke.points,
                    current,
                    controlOne,
                    controlTwo,
                    point
                );
                current = point;
                break;
            }
            case 'A':
            {
                const float radiusX = reader.readNumber();
                const float radiusY = reader.readNumber();
                const float rotation = reader.readNumber();
                const float largeArcValue = reader.readNumber();
                const float sweepValue = reader.readNumber();
                ImVec2 point = readPoint(reader);
                if ((largeArcValue != 0.0F && largeArcValue != 1.0F)
                    || (sweepValue != 0.0F && sweepValue != 1.0F))
                {
                    throw std::runtime_error(
                        "SVG arc flags must be zero or one");
                }
                if (relative)
                {
                    point = add(current, point);
                }
                appendArc(
                    stroke.points,
                    current,
                    point,
                    radiusX,
                    radiusY,
                    rotation,
                    largeArcValue != 0.0F,
                    sweepValue != 0.0F
                );
                current = point;
                break;
            }
            default:
                throw std::runtime_error(
                    std::string("unsupported SVG path command: ") + command);
            }
        }

        finishStroke();
        return strokes;
    }

    [[nodiscard]] std::string_view attribute(
        const std::string_view element,
        const std::string_view name)
    {
        std::size_t position = 0;
        while ((position = element.find(name, position))
            != std::string_view::npos)
        {
            const std::size_t afterName = position + name.size();
            const bool validBeginning = position == 0
                || std::isspace(static_cast<unsigned char>(
                    element[position - 1])) != 0
                || element[position - 1] == '<';
            std::size_t equals = afterName;
            while (equals < element.size()
                && std::isspace(static_cast<unsigned char>(element[equals])) != 0)
            {
                ++equals;
            }
            if (!validBeginning || equals == element.size()
                || element[equals] != '=')
            {
                position = afterName;
                continue;
            }

            ++equals;
            while (equals < element.size()
                && std::isspace(static_cast<unsigned char>(element[equals])) != 0)
            {
                ++equals;
            }
            if (equals == element.size()
                || (element[equals] != '"' && element[equals] != '\''))
            {
                throw std::runtime_error(
                    "SVG attribute is not quoted: " + std::string(name));
            }
            const char quote = element[equals++];
            const std::size_t end = element.find(quote, equals);
            if (end == std::string_view::npos)
            {
                throw std::runtime_error(
                    "unterminated SVG attribute: " + std::string(name));
            }
            return element.substr(equals, end - equals);
        }

        throw std::runtime_error(
            "missing SVG attribute: " + std::string(name));
    }

    [[nodiscard]] float scalarAttribute(
        const std::string_view element,
        const std::string_view name)
    {
        NumberReader reader(attribute(element, name));
        const float value = reader.readNumber();
        if (!reader.atEnd())
        {
            throw std::runtime_error(
                "unexpected value in SVG attribute: " + std::string(name));
        }
        return value;
    }

    void parseElements(
        const std::string_view svg,
        const std::string_view elementName,
        const auto& callback)
    {
        const std::string opening = "<" + std::string(elementName);
        std::size_t position = 0;
        while ((position = svg.find(opening, position))
            != std::string_view::npos)
        {
            const std::size_t end = svg.find('>', position);
            if (end == std::string_view::npos)
            {
                throw std::runtime_error(
                    "unterminated SVG element: " + std::string(elementName));
            }
            callback(svg.substr(position, end - position + 1));
            position = end + 1;
        }
    }

    [[nodiscard]] ParsedDrawing parseSvg(const std::string_view svg)
    {
        const std::size_t rootEnd = svg.find('>');
        if (rootEnd == std::string_view::npos
            || svg.substr(0, rootEnd).find("<svg") == std::string_view::npos)
        {
            throw std::runtime_error("missing SVG root element");
        }

        ParsedDrawing drawing;
        const std::string_view root = svg.substr(0, rootEnd + 1);
        NumberReader viewBoxReader(attribute(root, "viewBox"));
        drawing.viewBoxMinimum = readPoint(viewBoxReader);
        drawing.viewBoxSize = readPoint(viewBoxReader);
        if (!viewBoxReader.atEnd()
            || drawing.viewBoxSize.x <= 0.0F
            || drawing.viewBoxSize.y <= 0.0F)
        {
            throw std::runtime_error("invalid SVG viewBox");
        }
        drawing.strokeWidth = scalarAttribute(root, "stroke-width");
        if (drawing.strokeWidth <= 0.0F)
        {
            throw std::runtime_error("invalid SVG stroke width");
        }

        parseElements(svg, "path", [&](const std::string_view element)
        {
            std::vector<ParsedStroke> paths = parsePath(attribute(element, "d"));
            drawing.strokes.insert(
                drawing.strokes.end(),
                std::make_move_iterator(paths.begin()),
                std::make_move_iterator(paths.end())
            );
        });

        parseElements(svg, "circle", [&](const std::string_view element)
        {
            const ImVec2 center{
                scalarAttribute(element, "cx"),
                scalarAttribute(element, "cy")
            };
            const float radius = scalarAttribute(element, "r");
            if (radius <= 0.0F)
            {
                throw std::runtime_error("invalid SVG circle radius");
            }

            ParsedStroke circle;
            circle.closed = true;
            constexpr int circleSegments = 48;
            circle.points.reserve(circleSegments);
            for (int segment = 0; segment < circleSegments; ++segment)
            {
                const double angle = 2.0 * std::numbers::pi_v<double>
                    * static_cast<double>(segment)
                    / static_cast<double>(circleSegments);
                circle.points.push_back({
                    center.x + radius * static_cast<float>(std::cos(angle)),
                    center.y + radius * static_cast<float>(std::sin(angle))
                });
            }
            drawing.strokes.push_back(std::move(circle));
        });

        parseElements(svg, "line", [&](const std::string_view element)
        {
            drawing.strokes.push_back({{
                {
                    scalarAttribute(element, "x1"),
                    scalarAttribute(element, "y1")
                },
                {
                    scalarAttribute(element, "x2"),
                    scalarAttribute(element, "y2")
                }
            }, false});
        });

        parseElements(svg, "rect", [&](const std::string_view element)
        {
            const float x = scalarAttribute(element, "x");
            const float y = scalarAttribute(element, "y");
            const float width = scalarAttribute(element, "width");
            const float height = scalarAttribute(element, "height");
            const float radius = std::min({
                scalarAttribute(element, "rx"),
                width * 0.5F,
                height * 0.5F
            });
            if (width <= 0.0F || height <= 0.0F || radius < 0.0F)
            {
                throw std::runtime_error("invalid SVG rectangle");
            }

            ParsedStroke rectangle;
            rectangle.closed = true;
            constexpr int cornerSegments = 8;
            rectangle.points.reserve(cornerSegments * 4);
            const std::array<ImVec2, 4> centers{
                ImVec2{x + width - radius, y + radius},
                ImVec2{x + width - radius, y + height - radius},
                ImVec2{x + radius, y + height - radius},
                ImVec2{x + radius, y + radius}
            };
            for (std::size_t corner = 0; corner < centers.size(); ++corner)
            {
                const double startAngle = -std::numbers::pi_v<double> * 0.5
                    + static_cast<double>(corner)
                        * std::numbers::pi_v<double> * 0.5;
                for (int segment = 0; segment < cornerSegments; ++segment)
                {
                    const double angle = startAngle
                        + static_cast<double>(segment)
                            * std::numbers::pi_v<double> * 0.5
                            / static_cast<double>(cornerSegments - 1);
                    rectangle.points.push_back({
                        centers[corner].x
                            + radius * static_cast<float>(std::cos(angle)),
                        centers[corner].y
                            + radius * static_cast<float>(std::sin(angle))
                    });
                }
            }
            drawing.strokes.push_back(std::move(rectangle));
        });

        if (drawing.strokes.empty())
        {
            throw std::runtime_error("SVG contains no supported strokes");
        }

        const float maximumFloat = std::numeric_limits<float>::max();
        drawing.contentMinimum = {maximumFloat, maximumFloat};
        drawing.contentMaximum = {-maximumFloat, -maximumFloat};
        for (const ParsedStroke& stroke : drawing.strokes)
        {
            for (const ImVec2 point : stroke.points)
            {
                drawing.contentMinimum.x = std::min(
                    drawing.contentMinimum.x, point.x);
                drawing.contentMinimum.y = std::min(
                    drawing.contentMinimum.y, point.y);
                drawing.contentMaximum.x = std::max(
                    drawing.contentMaximum.x, point.x);
                drawing.contentMaximum.y = std::max(
                    drawing.contentMaximum.y, point.y);
            }
        }
        return drawing;
    }

    [[nodiscard]] std::string readFile(const std::filesystem::path& path)
    {
        std::ifstream input(path, std::ios::binary);
        if (!input)
        {
            throw std::runtime_error("file not found at " + path.string());
        }
        return {
            std::istreambuf_iterator<char>(input),
            std::istreambuf_iterator<char>()
        };
    }
}

namespace quantum::editor
{
    std::string_view editorIconFileName(const EditorIcon icon)
    {
        const std::size_t index = static_cast<std::size_t>(icon);
        if (index >= editorIconFileNames.size())
        {
            throw std::invalid_argument("Invalid editor icon.");
        }
        return editorIconFileNames[index];
    }

    void EditorIcons::load(const std::filesystem::path& basePath)
    {
        const std::filesystem::path iconDirectory =
            basePath / "assets/icons/lucide";
        std::array<Drawing, iconCount> loadedDrawings;

        for (std::size_t index = 0; index < loadedDrawings.size(); ++index)
        {
            const std::filesystem::path path = iconDirectory
                / editorIconFileName(static_cast<EditorIcon>(index));
            try
            {
                ParsedDrawing parsed = parseSvg(readFile(path));
                Drawing drawing;
                drawing.viewBoxMinimum = parsed.viewBoxMinimum;
                drawing.viewBoxSize = parsed.viewBoxSize;
                drawing.contentMinimum = parsed.contentMinimum;
                drawing.contentMaximum = parsed.contentMaximum;
                drawing.strokeWidth = parsed.strokeWidth;
                drawing.strokes.reserve(parsed.strokes.size());
                for (ParsedStroke& parsedStroke : parsed.strokes)
                {
                    drawing.strokes.push_back({
                        std::move(parsedStroke.points),
                        parsedStroke.closed
                    });
                }
                loadedDrawings[index] = std::move(drawing);
            }
            catch (const std::exception& error)
            {
                throw std::runtime_error(
                    "Could not load Lucide icon " + path.string()
                    + ": " + error.what());
            }
        }

        drawings_ = std::move(loadedDrawings);
        loaded_ = true;
    }

    void EditorIcons::clear() noexcept
    {
        drawings_ = {};
        loaded_ = false;
    }

    bool EditorIcons::button(
        const EditorIcon icon,
        const char* const id,
        const char* const tooltip,
        const bool selected,
        const bool enabled,
        const EditorIconRenderStyle& renderStyle) const
    {
        const EditorIconMetrics iconMetrics = metrics(icon, renderStyle);
        const ImGuiStyle& style = ImGui::GetStyle();

        if (!enabled)
        {
            ImGui::BeginDisabled();
        }
        const bool clicked = ImGui::InvisibleButton(
            id, ImVec2{iconMetrics.buttonExtent, iconMetrics.buttonExtent});
        const bool hovered = ImGui::IsItemHovered(
            ImGuiHoveredFlags_AllowWhenDisabled);
        const bool tooltipHovered = ImGui::IsItemHovered(
            ImGuiHoveredFlags_ForTooltip
                | ImGuiHoveredFlags_AllowWhenDisabled);
        const bool held = enabled && ImGui::IsItemActive();
        const ImVec2 minimum = ImGui::GetItemRectMin();
        const ImVec2 maximum = ImGui::GetItemRectMax();
        if (!enabled)
        {
            ImGui::EndDisabled();
        }

        ImDrawList* const drawList = ImGui::GetWindowDrawList();

        ImVec4 frameColor = palette::control;
        ImVec4 borderColor = palette::border;
        ImVec4 iconColor = palette::text;
        float borderThickness = style.FrameBorderSize;

        if (!enabled)
        {
            frameColor = palette::surfaceInset;
            iconColor = palette::textSecondary;
        }
        else if (selected)
        {
            frameColor = held
                ? palette::selectionActive
                : hovered ? palette::selectionHovered : palette::selection;
            borderColor = palette::accent;
            iconColor = palette::accent;
            const float presentationScale = std::max(
                style.FontScaleMain * style.FontScaleDpi, 0.5F);
            borderThickness = std::max(
                borderThickness,
                std::round(8.0F * presentationScale) * 0.25F
            );
        }
        else if (held)
        {
            frameColor = palette::selectionActive;
            borderColor = palette::accent;
            iconColor = palette::accent;
        }
        else if (hovered)
        {
            frameColor = palette::controlHovered;
            borderColor = palette::accent;
            iconColor = palette::accent;
        }

        drawList->AddRectFilled(
            minimum,
            maximum,
            ImGui::GetColorU32(frameColor),
            style.FrameRounding
        );
        if (borderThickness > 0.0F)
        {
            const float inset = borderThickness * 0.5F;
            drawList->AddRect(
                {minimum.x + inset, minimum.y + inset},
                {maximum.x - inset, maximum.y - inset},
                ImGui::GetColorU32(borderColor),
                std::max(0.0F, style.FrameRounding - inset),
                ImDrawFlags_None,
                borderThickness
            );
        }

        const Drawing& drawing =
            drawings_[static_cast<std::size_t>(icon)];
        const float drawingExtent = std::max(
            drawing.viewBoxSize.x, drawing.viewBoxSize.y);
        const float scale = iconMetrics.iconExtent / drawingExtent;
        const ImVec2 origin{
            minimum.x + iconMetrics.drawingMinimum.x
                - drawing.contentMinimum.x * scale,
            minimum.y + iconMetrics.drawingMinimum.y
                - drawing.contentMinimum.y * scale
        };
        const float radius = iconMetrics.strokeThickness * 0.5F;
        const ImU32 color = ImGui::GetColorU32(iconColor);

        for (const Stroke& stroke : drawing.strokes)
        {
            drawList->PathClear();
            for (const ImVec2 point : stroke.points)
            {
                drawList->PathLineTo({
                    origin.x + point.x * scale,
                    origin.y + point.y * scale
                });
            }
            drawList->PathStroke(
                color,
                stroke.closed ? ImDrawFlags_Closed : ImDrawFlags_None,
                iconMetrics.strokeThickness
            );

            // ImDrawList's polyline join is suitable at toolbar scale; add
            // explicit endpoint circles for Lucide's round open-path caps.
            if (!stroke.closed && !stroke.points.empty())
            {
                const auto addCap = [&](const ImVec2 point)
                {
                    drawList->AddCircleFilled({
                        origin.x + point.x * scale,
                        origin.y + point.y * scale
                    }, radius, color);
                };
                addCap(stroke.points.front());
                addCap(stroke.points.back());
            }
        }

        if (tooltipHovered)
        {
            ImGui::SetTooltip("%s", tooltip);
        }
        return enabled && clicked;
    }

    EditorIconMetrics EditorIcons::metrics(
        const EditorIcon icon,
        const EditorIconRenderStyle& renderStyle) const
    {
        if (!loaded_)
        {
            throw std::logic_error(
                "EditorIcons cannot draw before its SVGs are loaded.");
        }
        const std::size_t index = static_cast<std::size_t>(icon);
        if (index >= iconCount)
        {
            throw std::invalid_argument("Invalid editor icon.");
        }
        if (!std::isfinite(renderStyle.iconSize)
            || renderStyle.iconSize <= 0.0F
            || !std::isfinite(renderStyle.strokeThicknessMultiplier)
            || renderStyle.strokeThicknessMultiplier <= 0.0F)
        {
            throw std::invalid_argument(
                "Editor icon size and stroke multiplier must be positive.");
        }

        const ImGuiStyle& style = ImGui::GetStyle();
        const float presentationScale = std::max(
            style.FontScaleMain * style.FontScaleDpi, 0.5F);
        const Drawing& drawing = drawings_[index];
        const float drawingExtent = std::max(
            drawing.viewBoxSize.x, drawing.viewBoxSize.y);

        EditorIconMetrics result;
        result.iconExtent = std::max(
            1.0F, std::round(renderStyle.iconSize * presentationScale));
        result.buttonExtent = std::ceil(std::max(
            ImGui::GetFrameHeight(),
            result.iconExtent + 2.0F * style.FramePadding.y
        ));

        const float drawingScale = result.iconExtent / drawingExtent;
        const float unquantizedThickness = drawing.strokeWidth
            * drawingScale * renderStyle.strokeThicknessMultiplier;
        result.strokeThickness = std::max(
            0.25F, std::round(unquantizedThickness * 4.0F) * 0.25F);

        const ImVec2 contentSize{
            (drawing.contentMaximum.x - drawing.contentMinimum.x)
                * drawingScale,
            (drawing.contentMaximum.y - drawing.contentMinimum.y)
                * drawingScale
        };
        result.drawingMinimum = {
            (result.buttonExtent - contentSize.x) * 0.5F,
            (result.buttonExtent - contentSize.y) * 0.5F
        };
        result.drawingMaximum = {
            result.drawingMinimum.x + contentSize.x,
            result.drawingMinimum.y + contentSize.y
        };
        return result;
    }

    bool EditorIcons::loaded() const noexcept
    {
        return loaded_;
    }
}
