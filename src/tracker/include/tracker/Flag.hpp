#ifndef FLAG_H
#define FLAG_H
namespace allen
{
    class FLAG
    {
    public:
        enum Name
        {
            imshow,
            dataOn,
            initTarget,
            calibration,
            setRect,
            targetOn,
            reset
        };
        std::vector<std::pair<Name, bool> > data;
        std::vector<std::pair<Name, bool> > init_data;

    public:
        FLAG()
        {
            push_flag(FLAG::Name::imshow, true);
            push_flag(FLAG::Name::dataOn, false);
            push_flag(FLAG::Name::initTarget, false);
            push_flag(FLAG::Name::calibration, false);
            push_flag(FLAG::Name::setRect, false);
            push_flag(FLAG::Name::targetOn, false);
            push_flag(FLAG::Name::reset, false);
        }
        ~FLAG()
        {}
        void resetFlag(void)
        {
            data = init_data;
        }
        void push_flag(Name idx, bool _initStatus)
        {
            init_data.push_back(std::pair<Name, bool>(idx, _initStatus));
            data.push_back(std::pair<Name, bool>(idx, _initStatus));
        }
        void set_flag_on(Name idx)
        {
            data[idx].second = true;
        }
        void set_flag_off(Name idx)
        {
            data[idx].second = false;
        }
        bool get_flag(Name idx)
        {
            return data[idx].second;
        }
    };

}
#endif