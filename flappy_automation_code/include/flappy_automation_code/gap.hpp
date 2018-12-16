
struct Gap{
    float top;
    float bottom;
    float x_begin;
    float x_end;
    int weight;
    //Constructor
    Gap(float _top, float _bottom, float _begin, float _end):
    top(_top),bottom(_bottom),x_begin(_begin), x_end(_end){ 
                weight = 0;
    }
    //default constructor
    Gap(){
        top = 0.0f;
        bottom = 0.0f;
        x_begin = 0.0f;
        x_end = 0.0f;
        weight = 0;
    }
};